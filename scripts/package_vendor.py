#!/usr/bin/env python3
"""Package authorized vendor resources into local wheels consumed by Pixi.

Only maintainers updating vendor snapshots need this command. Installation uses
committed wheels; it never reads the original SDK or model directories.
"""
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import io
from pathlib import Path
import stat
from xml.etree import ElementTree
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo


ROOT = Path(__file__).resolve().parents[1]
TAG = "py3-none-linux_x86_64"
EXCLUDED_DIRECTORIES = {".git", "__pycache__", ".specstory"}


def urdf_paths(source: Path, prefix: str) -> tuple[str, ...]:
    paths: set[str] = set()
    for side in ("left", "right"):
        relative = f"{prefix}/{side}_sharpa_wave/{side}_sharpa_wave.urdf"
        paths.add(relative)
        root = ElementTree.parse(source / relative).getroot()
        for mesh in root.iter("mesh"):
            filename = mesh.attrib["filename"]
            if not filename.startswith("package://"):
                raise ValueError(f"Unsupported URDF mesh reference: {filename}")
            paths.add(f"{prefix}/{filename.removeprefix('package://')}")
    return tuple(sorted(paths))


def model_paths(source: Path) -> tuple[str, ...]:
    scene = "wave_01/dual_sharpa_wave/dual_sharpa_wave.xml"
    root = ElementTree.parse(source / scene).getroot()
    paths = {"LICENSE.txt", "NOTICE.txt", scene, *urdf_paths(source, "wave_01")}
    paths.update(f"wave_01/{mesh.attrib['file']}" for mesh in root.findall("asset/mesh"))
    return tuple(sorted(paths))


def write_wheel(source: Path, paths: tuple[str, ...], name: str, version: str,
                homepage: str, output: Path) -> Path:
    source = source.expanduser().resolve()
    selected: dict[str, Path] = {}
    for relative in paths:
        if Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError(f"Vendor resource path escapes its source directory: {relative}")
        path = source / relative
        if not path.exists():
            raise FileNotFoundError(f"Missing vendor resource: {path}")
        candidates = sorted(path.rglob("*")) if path.is_dir() else [path]
        for candidate in candidates:
            relative_path = candidate.relative_to(source)
            if (not candidate.is_file()
                    or EXCLUDED_DIRECTORIES.intersection(relative_path.parts)
                    or candidate.suffix in {".pyc", ".pyo"}):
                continue
            if not candidate.resolve().is_relative_to(source):
                raise ValueError(f"Vendor symlink escapes its source directory: {candidate}")
            selected[f"{name}/{relative_path.as_posix()}"] = candidate

    destination = output / f"{name}-{version}-{TAG}.whl"
    temporary = destination.with_suffix(".whl.tmp")
    metadata_root = f"{name}-{version}.dist-info"
    records: list[tuple[str, str, str]] = []

    def zip_info(path: str, mode: int = 0o644) -> ZipInfo:
        info = ZipInfo(path, date_time=(1980, 1, 1, 0, 0, 0))
        info.compress_type = ZIP_DEFLATED
        info.create_system = 3
        info.external_attr = (stat.S_IFREG | mode) << 16
        return info

    def record(path: str, digest: bytes, size: int) -> None:
        encoded = base64.urlsafe_b64encode(digest).rstrip(b"=").decode("ascii")
        records.append((path, f"sha256={encoded}", str(size)))

    output.mkdir(parents=True, exist_ok=True)
    try:
        with ZipFile(temporary, "w", compression=ZIP_DEFLATED, compresslevel=6) as archive:
            # Marker only: these packages contain data, never import an ABI-specific SDK.
            metadata = {
                f"{name}/__init__.py": b"",
                f"{metadata_root}/METADATA": (
                    "Metadata-Version: 2.1\n"
                    f"Name: {name.replace('_', '-')}\n"
                    f"Version: {version}\n"
                    "Summary: Authorized Sharpa Teleop vendor resource snapshot\n"
                    "License: See the original licenses bundled with the resources\n"
                    f"Project-URL: Upstream, {homepage}\n\n"
                ).encode(),
                f"{metadata_root}/WHEEL": (
                    "Wheel-Version: 1.0\n"
                    "Generator: sharpa-teleop-package-vendor\n"
                    "Root-Is-Purelib: false\n"
                    f"Tag: {TAG}\n"
                ).encode(),
            }
            for path, content in metadata.items():
                archive.writestr(zip_info(path), content)
                record(path, hashlib.sha256(content).digest(), len(content))
            for path, original in sorted(selected.items()):
                digest = hashlib.sha256()
                size = 0
                mode = stat.S_IMODE(original.stat().st_mode)
                with original.open("rb") as incoming, archive.open(zip_info(path, mode), "w") as outgoing:
                    while chunk := incoming.read(1024 * 1024):
                        outgoing.write(chunk)
                        digest.update(chunk)
                        size += len(chunk)
                record(path, digest.digest(), size)
            record_path = f"{metadata_root}/RECORD"
            rows = io.StringIO(newline="")
            writer = csv.writer(rows, lineterminator="\n")
            writer.writerows(records)
            writer.writerow((record_path, "", ""))
            archive.writestr(zip_info(record_path), rows.getvalue().encode())
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"{destination.name}: {len(selected)} resource files, {destination.stat().st_size:,} bytes")
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manus-sdk", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--wave-sdk", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "vendor")
    args = parser.parse_args()
    write_wheel(
        args.manus_sdk,
        (
            "License", "NOTICE.txt",
            "client/ManusSDK/include",
            "client/ManusSDK/lib/libManusSDK_Integrated.so",
            "retargeting_alg_release_V4.0/include/hand_retargeting_optimizer.so",
            "retargeting_alg_release_V4.0/include/hand_kinematic_casadi.so",
            "retargeting_alg_release_V4.0/include/VERSION.json",
            "retargeting_alg_release_V4.0/VERSION.json",
            "retargeting_alg_release_V4.0/requirements.txt",
            "retargeting_alg_release_V4.0/environment.yml",
        ) + urdf_paths(args.manus_sdk, "retargeting_alg_release_V4.0/urdf"),
        "sharpa_teleop_manus_resources", "4.0.0",
        "https://github.com/sharpa-robotics/sharpa-manus-sdk", args.output,
    )
    write_wheel(
        args.models, model_paths(args.models),
        "sharpa_teleop_model_resources", "1.0.0",
        "https://github.com/sharpa-robotics/sharpa-urdf-usd-xml", args.output,
    )
    # Keep the Wave SDK distribution intact, excluding generated caches only.
    write_wheel(
        args.wave_sdk, (".",), "sharpa_teleop_wave_resources", "5.0.11",
        "https://github.com/sharpa-robotics/sharpa-wave-sdk", args.output,
    )


if __name__ == "__main__":
    main()
