#!/usr/bin/env python3
"""
Check that the fingers, in the closed-jaw assembly pose, do not intersect the
supplied SCHUNK PSH 32-1 STL.

Requires: trimesh + manifold3d (pip install trimesh manifold3d rtree)
The SCHUNK STL is read in place; it is not copied or re-distributed.
"""

from __future__ import annotations

import json
import warnings
from pathlib import Path

import trimesh

warnings.filterwarnings("ignore", message="divide by zero encountered in divide")
warnings.filterwarnings("ignore", message="invalid value encountered in divide")

SCHUNK_STL = Path(
    r"E:\LeStoreDownload\dailypython\PARTserver0202610210621215433777210e898ce"
    r"\SCHUNK-0302132 PSH 32-1_ 0000.stl"
)


def main() -> None:
    base = Path(__file__).resolve().parent
    if not SCHUNK_STL.exists():
        raise FileNotFoundError(f"SCHUNK STL not found: {SCHUNK_STL}")

    body = trimesh.load(SCHUNK_STL, force="mesh")
    results = {"schunk_stl": str(SCHUNK_STL), "fingers": {}}
    for side in ("right", "left"):
        finger_path = base / "urdf" / f"finger_{side}_assembly.stl"
        finger = trimesh.load(finger_path, force="mesh")
        overlap = trimesh.boolean.intersection([body, finger], engine="manifold")
        volume = float(overlap.volume)
        results["fingers"][side] = {
            "finger_stl": str(finger_path.relative_to(base)),
            "intersection_volume_mm3": round(volume, 6),
            "pass": volume <= 1e-3,
        }

    results["pass"] = all(v["pass"] for v in results["fingers"].values())
    (base / "collision_check.json").write_text(
        json.dumps(results, indent=2), encoding="utf-8"
    )
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
