#!/usr/bin/env python3
"""
Independent STL checks for the gripper finger deliverables.

Run from this directory:
    python inspect_stl.py

Requires: trimesh (pip install trimesh)
"""

from __future__ import annotations

import json
from pathlib import Path

import trimesh

PETG_DENSITY_G_CM3 = 1.27
PA12CF_DENSITY_G_CM3 = 1.10

EXPECTED_PRINT_BBOX = (79.5, 150.0, 58.0)  # X, Y, Z, mm
TOL = 0.15


def inspect(path: Path, expected_bodies: int) -> dict:
    mesh = trimesh.load(path, force="mesh")
    bodies = mesh.split(only_watertight=False)
    bounds = mesh.bounds
    extent = mesh.extents
    volume_cm3 = float(mesh.volume) / 1000.0
    com = mesh.center_mass

    return {
        "file": path.name,
        "bytes": path.stat().st_size,
        "units": "mm (assumed; STL is unitless)",
        "triangles": int(len(mesh.faces)),
        "vertices": int(len(mesh.vertices)),
        "bodies_expected": expected_bodies,
        "bodies_found": len(bodies),
        "watertight": bool(mesh.is_watertight),
        "all_bodies_watertight": bool(all(b.is_watertight for b in bodies)),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "volume_cm3": round(volume_cm3, 3),
        "mass_petg_g": round(volume_cm3 * PETG_DENSITY_G_CM3, 1),
        "mass_pa12cf_g": round(volume_cm3 * PA12CF_DENSITY_G_CM3, 1),
        "extents_mm": [round(float(v), 4) for v in extent],
        "bounds_mm": {
            "min": [round(float(v), 4) for v in bounds[0]],
            "max": [round(float(v), 4) for v in bounds[1]],
        },
        "center_mass_mm": [round(float(v), 3) for v in com],
    }


def main() -> None:
    base = Path(__file__).resolve().parent
    files = [
        (base / "stl" / "finger_right_print.stl", 1),
        (base / "stl" / "finger_left_print.stl", 1),
        (base / "stl" / "fingers_pair_assembly.stl", 2),
        (base / "urdf" / "finger_right_assembly.stl", 1),
        (base / "urdf" / "finger_left_assembly.stl", 1),
    ]

    results = []
    for path, n in files:
        if not path.exists():
            raise FileNotFoundError(path)
        results.append(inspect(path, n))

    # Print-orientation checks.
    for r in results:
        if r["file"].endswith("_print.stl"):
            ext = r["extents_mm"]
            r["print_bbox_expected_mm"] = list(EXPECTED_PRINT_BBOX)
            r["print_bbox_ok"] = all(
                abs(ext[i] - EXPECTED_PRINT_BBOX[i]) <= TOL for i in range(3)
            )
            r["bed_origin_ok"] = (
                abs(r["bounds_mm"]["min"][0]) <= TOL
                and abs(r["bounds_mm"]["min"][1]) <= TOL
                and abs(r["bounds_mm"]["min"][2]) <= TOL
            )

    out = {
        "checks": results,
        "summary": {
            "all_watertight": all(r["all_bodies_watertight"] for r in results),
            "all_body_counts_ok": all(
                r["bodies_found"] == r["bodies_expected"] for r in results
            ),
            "all_print_oriented": all(
                r.get("print_bbox_ok", True) and r.get("bed_origin_ok", True)
                for r in results
            ),
        },
    }
    (base / "stl_checks.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"{'file':38s} {'bodies':>6s} {'watertight':>10s} {'vol cm3':>9s} {'mass PETG':>10s}")
    print("-" * 82)
    for r in results:
        print(
            f"{r['file']:38s} {r['bodies_found']:>6d} "
            f"{str(r['all_bodies_watertight']):>10s} {r['volume_cm3']:>9.2f} "
            f"{r['mass_petg_g']:>8.1f} g"
        )
    print("\nsummary:", json.dumps(out["summary"], indent=2))


if __name__ == "__main__":
    main()
