"""Compiled V4.0 objective with a native L-BFGS-B numerical solver.

The SDK still owns pose conversion, objective weights, warm-start state and
filtering. Generated SDK code lives only in a temporary build directory.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from typing import Any


class CompiledSolver:
    """Implement the V4.0 x0/p/lam_x0 solver contract without changing its cost."""

    def __init__(self, original: Any) -> None:
        import casadi as ca
        import numpy as np
        from scipy.optimize import minimize

        self._ca = ca
        self._np = np
        self._minimize = minimize
        self._original = original
        self._warmed = False
        self._stats: dict[str, Any] = {}
        self.last_error: Exception | None = None
        gradient = original.get_function("nlp_grad_f")
        if (
            gradient.size_in(0) != (22, 1)
            or gradient.size_in(1) != (394, 1)
            or gradient.size_out(0) != (1, 1)
            or gradient.size_out(1) != (22, 1)
            or gradient.nnz_in(0) != 22
            or gradient.nnz_in(1) != 394
            or gradient.nnz_out(0) != 1
            or gradient.nnz_out(1) != 22
            or original.numel_out("g") != 0
        ):
            raise RuntimeError("Unsupported SDK objective signature; expected the V4.0 WAVE model")

        self._build = tempfile.TemporaryDirectory(prefix="sharpa-retarget-")
        directory = Path(self._build.name)
        generator = ca.CodeGenerator("gradient.c")
        generator.add(gradient)
        generator.generate(str(directory) + os.sep)
        compiler = shlex.split(os.environ.get("CC", "cc"))
        if not compiler:
            raise RuntimeError("CC must name a C compiler")
        compiled = subprocess.run(
            [*compiler, "-O3", "-fPIC", "-shared", str(directory / "gradient.c"),
             "-lm", "-o", str(directory / "gradient.so")],
            capture_output=True, text=True, timeout=45,
        )
        if compiled.returncode:
            raise RuntimeError(f"Cannot compile retarget objective: {compiled.stderr}")
        self._library = ctypes.CDLL(str(directory / "gradient.so"))
        self._function = getattr(self._library, gradient.name())
        # Linux keeps the loaded mapping alive after unlink; even a terminated
        # SDK child must not leave generated proprietary objective files behind.
        self._build.cleanup()
        self._double_pointer = ctypes.POINTER(ctypes.c_double)
        integer_pointer = ctypes.POINTER(ctypes.c_longlong)
        self._function.argtypes = [
            ctypes.POINTER(self._double_pointer), ctypes.POINTER(self._double_pointer),
            integer_pointer, self._double_pointer, ctypes.c_int,
        ]
        self._function.restype = ctypes.c_int
        work = getattr(self._library, gradient.name() + "_work")
        work.argtypes = [integer_pointer] * 4
        work.restype = ctypes.c_int
        sizes = [ctypes.c_longlong() for _ in range(4)]
        if work(*[ctypes.byref(size) for size in sizes]) or any(size.value < 0 for size in sizes):
            raise RuntimeError("Invalid generated objective workspace")
        if sizes[0].value < 2 or sizes[1].value < 2:
            raise RuntimeError("Invalid generated objective argument slots")
        self._arguments = (self._double_pointer * sizes[0].value)()
        self._results = (self._double_pointer * sizes[1].value)()
        self._integer_work = np.empty(sizes[2].value, dtype=np.int64)
        self._real_work = np.empty(sizes[3].value, dtype=np.float64)
        self._integer_pointer = self._integer_work.ctypes.data_as(integer_pointer)
        self._real_pointer = self._real_work.ctypes.data_as(self._double_pointer)
        self._value = ctypes.c_double()
        self._gradient = np.empty(22, dtype=np.float64)
        self._results[0] = ctypes.pointer(self._value)
        self._results[1] = self._gradient.ctypes.data_as(self._double_pointer)
        self._zero_multipliers = ca.DM.zeros(22)

    def _evaluate(self, angles: Any) -> tuple[float, Any]:
        self._arguments[0] = angles.ctypes.data_as(self._double_pointer)
        status = self._function(
            self._arguments, self._results, self._integer_pointer, self._real_pointer, 0
        )
        if status:
            raise RuntimeError(f"Compiled objective failed with status {status}")
        if not self._np.isfinite(self._value.value) or not self._np.isfinite(self._gradient).all():
            # A line-search trial may leave the exponential penalties' domain.
            # Reject that trial; never permit it to become an output solution.
            return float("inf"), self._np.zeros(22)
        # SciPy retains gradients across calls; it must not alias our C workspace.
        return self._value.value, self._gradient.copy()

    def __call__(self, **inputs: Any) -> dict[str, Any]:
        self.last_error = None
        try:
            return self._solve(inputs)
        except Exception as error:
            self.last_error = error
            raise

    def _solve(self, inputs: dict[str, Any]) -> dict[str, Any]:
        np = self._np
        if set(inputs) - {"x0", "p", "lam_x0"}:
            raise RuntimeError("Unexpected SDK solver inputs; refusing to change constraint semantics")
        initial = np.ascontiguousarray(inputs["x0"], dtype=np.float64).reshape(-1)
        parameter = np.ascontiguousarray(inputs["p"], dtype=np.float64).reshape(-1)
        if initial.shape != (22,) or parameter.shape != (394,):
            raise ValueError("Invalid SDK solver input shape")
        if not np.isfinite(initial).all() or not np.isfinite(parameter).all():
            raise ValueError("Non-finite SDK solver input")
        if not self._warmed:
            # The original solver establishes a robust first pose from rest.
            # This is a genuine solve of the first request, not a synthetic seed.
            result = self._native_solve(inputs)
            self._warmed = True
            return result

        self._arguments[1] = parameter.ctypes.data_as(self._double_pointer)
        result = self._minimize(
            self._evaluate, initial, jac=True, method="L-BFGS-B",
            options={"maxiter": 20, "maxls": 50, "ftol": 1e-9, "gtol": 1e-5},
        )
        if not result.success and result.status != 1:
            # Recover numerical line-search failures with a real solve of the
            # same frame, rather than silently re-emitting the previous pose.
            return self._native_solve(inputs)
        value, _ = self._evaluate(result.x)
        if not np.isfinite(value) or not np.isfinite(result.x).all():
            raise RuntimeError("Optimizer selected a non-finite solution")
        # V4.0 uses this status to retain x/dq warm-start state. Preserve the
        # distinction between convergence and a usable iteration-limited solve.
        status = "Solve_Succeeded" if result.success else "Maximum_Iterations_Exceeded"
        self._stats = {
            "success": bool(result.success), "return_status": status,
            "iter_count": int(result.nit), "n_call_nlp_f": int(result.nfev),
        }
        return {"x": self._ca.DM(result.x), "f": self._ca.DM(value),
                "lam_x": self._zero_multipliers}

    def _native_solve(self, inputs: dict[str, Any]) -> dict[str, Any]:
        result = self._original(**inputs)
        self._stats = self._original.stats()
        accepted = self._stats.get("success") or self._stats.get("return_status") in {
            "Maximum_Iterations_Exceeded", "Solved_To_Acceptable_Level",
        }
        if not accepted or not self._np.isfinite(result["x"]).all() or not self._np.isfinite(float(result["f"])):
            raise RuntimeError(f"Native optimizer failed: {self._stats.get('return_status')}")
        return result

    def reset(self, original: Any) -> None:
        """Reuse compiled code when the SDK resets its numerical state."""
        if original.get_function("nlp_grad_f").serialize() != self._original.get_function("nlp_grad_f").serialize():
            raise RuntimeError("SDK reset changed the compiled objective")
        self._original = original
        self._warmed = False
        self.last_error = None

    def stats(self) -> dict[str, Any]:
        return self._stats


def optimized_worker(sdk_root: str, ready: Any, *args: Any, **kwargs: Any) -> None:
    """Configure only this native worker process, before entering its SDK loop."""
    from .retarget_worker import _prepare_upstream_imports

    side = args[0]
    try:
        _prepare_upstream_imports(sdk_root)
        import hand_retargeting_optimizer as vendor

        optimizer_type = vendor.HandRetargetingOptimizer
        original_init = optimizer_type.__init__
        original_optimize = optimizer_type.optimize_with_casadi
        original_initialize = optimizer_type._initialize_casadi_optimizer

        def initialize_solver(self: Any) -> None:
            original_initialize(self)
            compiled = getattr(self, "_compiled_solver", None)
            if compiled is None:
                compiled = CompiledSolver(self.casadi_solver)
                self._compiled_solver = compiled
            else:
                compiled.reset(self.casadi_solver)
            self.casadi_solver = compiled

        def initialize(self: Any, *arguments: Any, **options: Any) -> None:
            try:
                original_init(self, *arguments, **options)
                self._initialize_casadi_optimizer()
            except Exception as error:
                ready.put((side, str(error)))
                raise
            ready.put((side, None))

        def optimize(self: Any, *arguments: Any, **options: Any) -> Any:
            result = original_optimize(self, *arguments, **options)
            # The vendor method can catch solver exceptions and return an old
            # angle vector. Surface those failures to the native manager instead.
            if self.casadi_solver.last_error is not None:
                raise RuntimeError("Retarget numerical solve failed") from self.casadi_solver.last_error
            return result

        optimizer_type.__init__ = initialize
        optimizer_type._initialize_casadi_optimizer = initialize_solver
        optimizer_type.optimize_with_casadi = optimize
        vendor.optimization_worker_multiprocess(*args, **kwargs)
    except Exception as error:
        ready.put((side, str(error)))
        raise
