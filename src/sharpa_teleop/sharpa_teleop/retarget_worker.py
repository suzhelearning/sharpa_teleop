"""Python 3.10 subprocess for the proprietary Sharpa retargeting optimizer.

Its stdout is a JSON-lines protocol only.  The upstream optimizer and all of its
children inherit stderr for diagnostics so they cannot corrupt that protocol.
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any, TextIO


_HAND_SIZE = 25
_JOINT_COUNT = 22

_SIDES = ("left", "right")
_STOP = object()


def _redirect_non_protocol_stdout() -> TextIO:
    """Keep a duplicate protocol pipe while directing upstream output to stderr."""
    sys.stdout.flush()
    protocol_fd = os.dup(sys.stdout.fileno())
    os.set_inheritable(protocol_fd, False)
    protocol = os.fdopen(protocol_fd, "w", encoding="utf-8", buffering=1)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    return protocol


def _emit(
    protocol: TextIO,
    message: dict[str, Any],
    lock: Any | None = None,
) -> None:
    payload = json.dumps(message, allow_nan=False, separators=(",", ":")) + "\n"
    if lock is None:
        protocol.write(payload)
        protocol.flush()
        return
    with lock:
        protocol.write(payload)
        protocol.flush()


def _retarget_root(sdk_root: str) -> Path:
    if not sdk_root.strip():
        raise RuntimeError("sdk_root is empty; set SHARPA_MANUS_SDK or the sdk_root parameter")
    root = Path(sdk_root).expanduser().resolve()
    release_root = root / "retargeting_alg_release_V4.0"
    if release_root.is_dir():
        root = release_root
    if not (root / "include" / "hand_retargeting_optimizer.so").is_file():
        raise RuntimeError(
            "sdk_root does not contain retargeting_alg_release_V4.0/include/"
            "hand_retargeting_optimizer.so"
        )
    if not (root / "urdf").is_dir():
        raise RuntimeError(f"Missing upstream URDF directory under {root}")
    return root


def _prepare_upstream_imports(sdk_root: str) -> tuple[Any, Any, Any, Any]:
    """Import the ABI-specific optimizer after placing its URDF-relative cwd first."""
    root = _retarget_root(sdk_root)
    os.chdir(root)
    include = root / "include"
    include_text = str(include)
    if include_text not in sys.path:
        sys.path.insert(0, include_text)
    package_root = str(Path(__file__).resolve().parents[1])
    if package_root not in sys.path:
        sys.path.insert(0, package_root)

    try:
        import numpy as np
        from hand_retargeting_optimizer import MultiprocessOptimizationManager, init_hand_model
    except Exception as error:
        raise RuntimeError(
            "Failed to import the upstream hand_retargeting_optimizer with the supplied Python; "
            "use the Python 3.10 retarget environment"
        ) from error
    return np, MultiprocessOptimizationManager, init_hand_model, root


def _validate_request(request: Any, np: Any) -> tuple[str, int, Any]:
    if not isinstance(request, dict):
        raise ValueError("request must be a JSON object")
    side = request.get("side")
    frame = request.get("frame")
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    if isinstance(frame, bool) or not isinstance(frame, int) or frame < 0:
        raise ValueError("frame must be a non-negative integer")

    try:
        points = np.asarray(request.get("points"), dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError("points must contain numeric values") from error
    if points.shape != (_HAND_SIZE, 7):
        raise ValueError(f"points must have shape ({_HAND_SIZE}, 7), got {points.shape}")
    if not bool(np.isfinite(points).all()):
        raise ValueError("points contain non-finite values")
    if bool(np.all(points == 0.0)):
        raise ValueError("points represent an inactive all-zero hand")
    if bool(np.any(np.sum(points[:, 3:] * points[:, 3:], axis=1) == 0.0)):
        raise ValueError("points contain a zero quaternion")
    return side, frame, points


def _validate_models(hand_models: Any) -> dict[str, list[str]]:
    if not isinstance(hand_models, dict):
        raise RuntimeError("init_hand_model('WAVE') did not return side-indexed hand models")

    names_by_side: dict[str, list[str]] = {}
    for side in ("left", "right"):
        hand_model = hand_models.get(side)
        if hand_model is None:
            raise RuntimeError(f"init_hand_model('WAVE') did not provide the {side} hand model")
        try:
            names = list(hand_model.joint_names)
        except Exception as error:
            raise RuntimeError(f"Upstream {side} hand model does not expose joint_names") from error
        if len(names) != _JOINT_COUNT or not all(isinstance(name, str) and name for name in names):
            raise RuntimeError(
                f"Upstream {side} hand model must expose {_JOINT_COUNT} non-empty joint names; got {len(names)}"
            )
        names_by_side[side] = names
    return names_by_side


def _shutdown_manager(manager: Any) -> None:
    """Run both documented lifecycle calls so optimizer children do not outlive us."""
    segments = [
        shared.shm
        for shared in (manager.left_shm_manager, manager.right_shm_manager)
        if shared is not None and shared.shm is not None
    ]
    try:
        manager.stop()
    except Exception:
        traceback.print_exc(file=sys.stderr)
    try:
        manager.cleanup()
    except Exception:
        traceback.print_exc(file=sys.stderr)
    # V4.0 stop() drops its shared-memory managers before cleanup() can unlink.
    for segment in segments:
        try:
            segment.unlink()
        except FileNotFoundError:
            pass
        finally:
            segment.close()


def _process_request(
    request: Any,
    manager: Any,
    names_by_side: dict[str, list[str]],
    np: Any,
) -> dict[str, Any]:
    side, frame, points = _validate_request(request, np)

    deadline = time.monotonic() + 10.0
    while not manager.update_process_keypoints(side, points, frame):
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Optimizer did not accept {side} frame {frame}")
        time.sleep(0.0001)

    # V4.0 get_result() is a shared-memory snapshot, NOT a completion wait.
    # Input submission updates frame_index before the result is computed.
    # Wait on both input/processing flags under the vendor's input lock so
    # neither initial zeros nor a previous pose acquire the new frame stamp.
    shared = getattr(manager, f"{side}_shm_manager")
    while True:
        with shared.input_lock:
            complete = (
                shared.arr[shared.has_new_data_offset] == 0
                and shared.arr[shared.is_processing_offset] == 0
            )
        if complete:
            break
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Optimizer did not complete {side} frame {frame}")
        if any(not process.is_alive() for process in manager.processes):
            raise RuntimeError("An optimizer subprocess exited")
        time.sleep(0.0001)

    result = manager.get_result(side)
    if result is None or not result.success or not math.isfinite(result.cost_value):
        raise RuntimeError(f"Optimizer failed for {side} frame {frame}")
    if not hasattr(result, "frame_index") or int(result.frame_index) != frame:
        actual_frame = getattr(result, "frame_index", None)
        raise RuntimeError(
            f"Optimizer returned {side} frame {actual_frame!r} while waiting for frame {frame}"
        )
    if not hasattr(result, "filtered_angles"):
        raise RuntimeError("Optimizer result does not expose filtered_angles")

    positions = np.asarray(result.filtered_angles, dtype=np.float64)
    if positions.shape != (_JOINT_COUNT,) or not bool(np.isfinite(positions).all()):
        raise RuntimeError(
            f"Optimizer returned invalid {side} joint positions with shape {positions.shape}"
        )

    # joint_names comes directly from init_hand_model('WAVE') and positions are
    # the upstream filtered angles, whose demo emits as radians.
    return {
        "side": side,
        "frame": frame,
        "names": names_by_side[side],
        "positions": [float(value) for value in positions],
    }


def _emit_request_error(
    protocol: TextIO,
    protocol_lock: Any,
    request: Any,
    error: Exception | str,
) -> None:
    side = request.get("side") if isinstance(request, dict) else None
    frame = request.get("frame") if isinstance(request, dict) else None
    _emit(
        protocol,
        {
            "type": "error",
            "stage": "request",
            "side": side,
            "frame": frame,
            "error": str(error),
        },
        protocol_lock,
    )


def _solve_requests(
    requests: queue.Queue[Any],
    manager: Any,
    names_by_side: dict[str, list[str]],
    np: Any,
    protocol: TextIO,
    protocol_lock: Any,
) -> None:
    """Solve one side's FIFO serially while the other side runs independently."""
    while True:
        request = requests.get()
        if request is _STOP:
            return
        try:
            response = _process_request(request, manager, names_by_side, np)
            _emit(protocol, response, protocol_lock)
        except Exception as error:
            _emit_request_error(protocol, protocol_lock, request, error)
            traceback.print_exc(file=sys.stderr)


def _enqueue_request(
    request: Any,
    requests_by_side: dict[str, queue.Queue[Any]],
    protocol: TextIO,
    protocol_lock: Any,
) -> None:
    side = request.get("side") if isinstance(request, dict) else None
    if side not in _SIDES:
        _emit_request_error(protocol, protocol_lock, request, "side must be left or right")
        return
    try:
        requests_by_side[side].put_nowait(request)
    except queue.Full:
        # The parent permits one unsolved request per side.  Reject rather than
        # silently replacing it, so every accepted source frame receives the
        # matching result or an explicit error.
        _emit_request_error(
            protocol,
            protocol_lock,
            request,
            f"Optimizer already has an outstanding {side} frame",
        )


def run(sdk_root: str, protocol: TextIO) -> int:
    manager: Any | None = None
    protocol_lock = threading.Lock()
    requests_by_side = {side: queue.Queue(maxsize=1) for side in _SIDES}
    solver_threads: list[threading.Thread] = []
    try:
        # The upstream multiprocess demo requires spawn before creating its manager.
        mp.set_start_method("spawn", force=True)
        np, manager_type, init_hand_model, _ = _prepare_upstream_imports(sdk_root)
        hand_models = init_hand_model("WAVE")
        names_by_side = _validate_models(hand_models)
        manager = manager_type(hand_models, filter_alpha=0.2, hand_serial="WAVE")
        # Use the SDK's original IPOPT workers without replacing its solver.
        # _process_request waits for actual completion before publishing results.
        manager.start()
        for side in _SIDES:
            thread = threading.Thread(
                target=_solve_requests,
                args=(
                    requests_by_side[side],
                    manager,
                    names_by_side,
                    np,
                    protocol,
                    protocol_lock,
                ),
                name=f"sharpa-retarget-{side}",
            )
            thread.start()
            solver_threads.append(thread)
        _emit(protocol, {"type": "ready"}, protocol_lock)

        for line in sys.stdin:
            if not line.strip():
                continue
            request: Any = None
            try:
                request = json.loads(line)
                _enqueue_request(request, requests_by_side, protocol, protocol_lock)
            except Exception as error:
                _emit_request_error(protocol, protocol_lock, request, error)
                traceback.print_exc(file=sys.stderr)
        return 0
    except Exception as error:
        _emit(protocol, {"type": "error", "stage": "startup", "error": str(error)}, protocol_lock)
        traceback.print_exc(file=sys.stderr)
        return 1
    finally:
        # EOF from the parent is a clean drain: sentinels follow already accepted
        # frames, preserving a response for each one before manager cleanup.
        for requests in requests_by_side.values():
            requests.put(_STOP)
        for thread in solver_threads:
            thread.join()
        if manager is not None:
            _shutdown_manager(manager)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-root", required=True)
    args = parser.parse_args(argv)
    protocol = _redirect_non_protocol_stdout()
    try:
        return run(args.sdk_root, protocol)
    finally:
        protocol.close()


if __name__ == "__main__":
    raise SystemExit(main())
