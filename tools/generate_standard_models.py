#!/usr/bin/env python3
"""Recreate and verify the six dimensioned STEP fixtures and a seeded texture."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import struct
import subprocess
import sys
import zlib

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data" / "performance-validation"
EXECUTABLE = ROOT / "apps" / "ParamCAD Studio" / "build" / "msvc-debug" / "Release" / "cad_reconstruction_validation.exe"
CASES = ("box-through-hole", "l-bracket-gusset", "sphere", "cylinder", "cone", "torus")


def create_texture(path: Path, seed: int = 4500, size: int = 1024) -> None:
    """Deterministic multiscale random squares; stdlib only, no downloaded asset."""
    rng = random.Random(seed)
    # Large irregular tiles give distinctive keypoints; fine tiles add detail.
    coarse = [[rng.randrange(50, 230) for _ in range(32)] for _ in range(32)]
    fine = [[rng.randrange(-45, 46) for _ in range(256)] for _ in range(256)]
    pixels = bytearray()
    for y in range(size):
        pixels.append(0)  # PNG filter: none
        for x in range(size):
            value = max(20, min(245, coarse[y * 32 // size][x * 32 // size]
                               + fine[y * 256 // size][x * 256 // size]))
            pixels.extend((value, value, value))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))

    data = b"\x89PNG\r\n\x1a\n"
    data += chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
    data += chunk(b"IDAT", zlib.compress(pixels, 9)) + chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, default=EXECUTABLE)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=CASES)
    args = parser.parse_args()
    if not args.executable.is_file():
        parser.error("Build with -DCAD_CLIENT_BUILD_VALIDATION_TOOLS=ON first")
    for name in args.cases:
        directory = DATA / name
        model = directory / "model.step"
        report = directory / "model_report.json"
        log = ROOT / "outputs" / f"generate-{name}.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        with log.open("w", encoding="utf-8") as stream:
            result = subprocess.run([str(args.executable), "generate", str(directory / "case.json"),
                                     str(model), str(report)], stdout=stream, stderr=subprocess.STDOUT)
        diagnostic = json.loads(report.read_text(encoding="utf-8")) if report.is_file() else {"status": "no_report"}
        if result.returncode:
            print(json.dumps({"case": name, **diagnostic, "log": str(log)}))
            return result.returncode
        diagnostic["model_sha256"] = hashlib.sha256(model.read_bytes()).hexdigest()
        report.write_text(json.dumps(diagnostic, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"case": name, "status": diagnostic["status"],
                          "bounds_mm": diagnostic["bounding_box_mm"], "volume_mm3": diagnostic["volume_mm3"],
                          "solids": diagnostic["solids"]}))
    texture = DATA / "textures" / "standard_capture.png"
    create_texture(texture)
    print(json.dumps({"texture": str(texture.relative_to(ROOT)), "seed": 4500,
                      "size": [1024, 1024], "sha256": hashlib.sha256(texture.read_bytes()).hexdigest()}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
