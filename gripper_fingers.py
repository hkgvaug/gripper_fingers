#!/usr/bin/env python3
"""
Parametric SCHUNK PSH 32-1 gripper fingers (pair).

Everything is in millimetres.  The script builds the fingers in the same
coordinate system as the supplied SCHUNK STL:
    X = jaw opening direction (outer jaw faces at X = +/-85.5 mm when closed)
    Y = depth of the gripper (jaw mounting faces are centred at Y = 31.5 mm)
    Z = vertical / finger-length direction
so an exported assembly STL overlays the original gripper STL with no offset.

The fingers mount on the OUTER faces of the two base jaws, as in the reference
photo (socket-head screws visible from outside).  Each finger drops below the
gripper body and then steps inward, so it clears the four exposed guide rods
and the central housing.  The right-hand finger is the nominal part.  The left-hand finger is its mirror
image.  Print files are additionally rotated so that the broad side of the
finger lies on the print bed at Z = 0 (no supports).

This script does NOT modify or re-export the SCHUNK gripper body.
"""

from __future__ import annotations

import json
from pathlib import Path

import cadquery as cq

# =============================================================================
# 1. USER PARAMETERS  (all lengths mm, angles degrees)
# =============================================================================

# ---- Overall finger envelope -------------------------------------------------
FINGER_LENGTH = 150.0          # root top to tip; SCHUNK limit is 170 mm
FINGER_WIDTH = 58.0            # constant Y width of the finger
ROOT_WIDTH = 58.0              # root plate width in Y (must fit the 60 mm jaw face)
ROOT_HEIGHT = 60.0             # root plate height in Z before the lower leg
ROOT_THICKNESS = 14.0          # X thickness of the root plate against the jaw
BLADE_THICKNESS = 16.0         # X thickness of the long grip blade
ARM_HEIGHT = 12.0              # Z height of the horizontal offset arm
ARM_CLEARANCE_BELOW_BODY = 2.0 # clearance between arm top and gripper body bottom

# ---- Finger spacing ----------------------------------------------------------
# Placeholder: the required wood-member width was not stated by the user.
WORKPIECE_WIDTH_PLACEHOLDER = 40.0  # CONFIRM
GRIP_GAP_CLOSED = WORKPIECE_WIDTH_PLACEHOLDER  # blade-to-blade gap, closed, before pads

# ---- Pads --------------------------------------------------------------------
PAD_THICKNESS = 2.0            # rubber pad / recess depth; placeholder
PAD_WIDTH = 36.0               # Y width of the bonding flat
PAD_LENGTH = 60.0              # Z length of the bonding flat
PAD_TIP_MARGIN = 8.0           # blade material left below the pad
PAD_CORNER_R = 4.0

# ---- Edges / stress relief ---------------------------------------------------
BLADE_FILLET_R = 6.0           # concave blade-to-arm fillet
ARM_ROOT_FILLET_R = 2.0        # small gusset at arm/root; must stay below jaw
EDGE_CHAMFER = 0.8             # print-friendly chamfer on root outer face and tip

# Guide-rod reliefs on the mounting face.  The PSH 32-1 guide rods protrude
# about 2 mm beyond the outer jaw faces; these rectangular pockets keep the
# root flat on the machined jaw face.  List = (Y, Z, width_Y, height_Z),
# relative to the face centre (Y=31.5, Z=0) in the finger's local frame.
# All four positions are cut in both fingers so the mirrored part is covered.
ROD_RELIEF_YZ_SIZE = [
    (-12.5, 19.0, 11.5, 11.5),
    (-12.5, -19.0, 11.5, 11.5),
    (18.5, 18.0, 16.0, 16.0),
    (18.5, -18.0, 16.0, 16.0),
]
ROD_RELIEF_DEPTH = 2.3
ROD_RELIEF_FACE_MARGIN = 0.5

# ---- Fasteners ---------------------------------------------------------------
# The SCHUNK manual identifies the base-jaw finger connection as M5.
# The STL shows 4 through holes (dia 4.92 mm) with counterbores on the outer
# jaw faces.  The printed root sits on the outer face; screws pass through the
# root from outside and thread into the M5 threads in the base jaw.
MOUNT_THREAD = "M5"
MOUNT_SCREW = "M5 x 16 socket-head cap screw (SHCS)"
MOUNT_HOLE_COMP = 0.2          # default clearance compensation, mm
MOUNT_CLEARANCE_D = 5.5        # M5 normal clearance before compensation
MOUNT_CBORE_D = 9.0            # socket-head counterbore before compensation
MOUNT_CBORE_DEPTH = 5.0
MOUNT_HOLE_PATTERN_YZ = [      # relative to the jaw-face centre (Y=31.5, Z=0)
    (-20.0, 0.0),
    (0.0, 20.0),
    (20.0, 0.0),
    (0.0, -20.0),
]

# ---- Measured SCHUNK PSH 32-1 interface (from supplied STL, mm) --------------
JAW_MOUNT_FACE_X = 85.5        # OUTER jaw face, right-hand jaw
JAW_CENTER_Y = 31.5
JAW_FACE_Y_MIN = 1.5
JAW_FACE_Y_MAX = 61.5
JAW_FACE_Z_MIN = -36.5
JAW_FACE_Z_MAX = 36.5
JAW_BODY_BOTTOM_Z = -38.0      # body underside; the arm must pass below this
JAW_BODY_HALF_WIDTH_X = 48.5   # body reaches X = +/-48.5
JAW_GAP_CLOSED_AT_MOUNTS = 171.0   # X=-85.5 to X=+85.5 (outer faces, CAD pose)
JAW_STROKE_PER_JAW = 35.0
JAW_CLOSING_FORCE_TOTAL = 750.0    # N, arithmetic sum of both jaws (375 N/jaw)

# ---- Official SCHUNK limits / reference data ---------------------------------
MAX_PERMITTED_FINGER_LENGTH = 170.0  # mm, at nominal 6 bar
MAX_PERMITTED_FINGER_WEIGHT = 1.5     # kg per finger
MAX_MX = 20.0                         # Nm per base jaw
MAX_MY = 10.0
MAX_MZ = 10.0
MAX_FZ = 300.0                        # N
NOMINAL_PRESSURE_BAR = 6.0

# ---- Material densities for the report ---------------------------------------
PETG_DENSITY_G_CM3 = 1.27
PA12CF_DENSITY_G_CM3 = 1.10

# ---- Output / mesh quality ---------------------------------------------------
STL_TOLERANCE = 0.05
STL_ANGULAR_TOLERANCE = 0.2

# =============================================================================
# 2. DERIVED GEOMETRY  (do not edit unless changing the design intent)
# =============================================================================

ROOT_FRONT_X = JAW_MOUNT_FACE_X + ROOT_THICKNESS
GRIP_OFFSET_TOTAL = (JAW_GAP_CLOSED_AT_MOUNTS - GRIP_GAP_CLOSED) / 2.0
BLADE_INNER_X = JAW_MOUNT_FACE_X - GRIP_OFFSET_TOTAL
BLADE_OUTER_X = BLADE_INNER_X + BLADE_THICKNESS
ARM_LENGTH_X = JAW_MOUNT_FACE_X - BLADE_OUTER_X

ROOT_TOP_Z = ROOT_HEIGHT / 2.0
ROOT_BOTTOM_Z = ROOT_TOP_Z - ROOT_HEIGHT
ARM_TOP_Z = JAW_BODY_BOTTOM_Z - ARM_CLEARANCE_BELOW_BODY
ARM_BOTTOM_Z = ARM_TOP_Z - ARM_HEIGHT
ROOT_LEG_BOTTOM_Z = ARM_BOTTOM_Z
TIP_Z = ROOT_TOP_Z - FINGER_LENGTH
BLADE_LENGTH = ARM_TOP_Z - TIP_Z

PAD_Z0 = TIP_Z + PAD_TIP_MARGIN
PAD_Z1 = PAD_Z0 + PAD_LENGTH
PAD_CENTER_Z = (PAD_Z0 + PAD_Z1) / 2.0

CLEARANCE_D = MOUNT_CLEARANCE_D + MOUNT_HOLE_COMP
CBORE_D = MOUNT_CBORE_D + MOUNT_HOLE_COMP


def validate_parameters() -> None:
    """Fail early if a parameter combination is not manufacturable/designed."""
    if FINGER_LENGTH > MAX_PERMITTED_FINGER_LENGTH:
        raise ValueError(
            f"FINGER_LENGTH={FINGER_LENGTH} exceeds SCHUNK PSH 32-1 limit "
            f"{MAX_PERMITTED_FINGER_LENGTH} mm"
        )
    if ROOT_WIDTH > (JAW_FACE_Y_MAX - JAW_FACE_Y_MIN):
        raise ValueError("ROOT_WIDTH does not fit the measured 60 mm jaw face")
    if ROOT_TOP_Z > JAW_FACE_Z_MAX or (ROOT_TOP_Z - ROOT_HEIGHT) < JAW_FACE_Z_MIN:
        raise ValueError("Root plate is not contained by the measured jaw face in Z")
    if JAW_MOUNT_FACE_X <= JAW_BODY_HALF_WIDTH_X:
        raise ValueError("Outer jaw face must be outside the central body")
    if ARM_LENGTH_X <= 0:
        raise ValueError("Inward offset too small: arm length is non-positive")
    if ARM_TOP_Z >= JAW_BODY_BOTTOM_Z:
        raise ValueError("Arm does not clear the gripper body")
    if BLADE_INNER_X <= JAW_BODY_HALF_WIDTH_X and TIP_Z > JAW_BODY_BOTTOM_Z:
        raise ValueError("Blade would intersect the gripper body")
    if TIP_Z >= ARM_TOP_Z:
        raise ValueError("Blade has no length")
    if PAD_Z0 <= TIP_Z or PAD_Z1 >= ARM_TOP_Z:
        raise ValueError("Pad pocket does not fit on the blade")
    if PAD_WIDTH > FINGER_WIDTH or PAD_THICKNESS >= BLADE_THICKNESS:
        raise ValueError("Pad geometry does not fit the blade")
    if CBORE_D / 2.0 > min(ROOT_WIDTH / 2.0, ROOT_HEIGHT / 2.0):
        raise ValueError("Counterbore is too large for the root plate")
    if MOUNT_CBORE_DEPTH >= ROOT_THICKNESS:
        raise ValueError("Counterbore breaks through the root plate")
    for y, z in MOUNT_HOLE_PATTERN_YZ:
        if abs(y) + CBORE_D / 2.0 > ROOT_WIDTH / 2.0:
            raise ValueError("Mounting hole/counterbore outside root width")
        if abs(z) + CBORE_D / 2.0 > ROOT_HEIGHT / 2.0:
            raise ValueError("Mounting hole/counterbore outside root height")


def box_xyz(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float):
    """Axis-aligned box from min/max coordinates."""
    return (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, z1 - z0)
        .translate(((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0))
    )


def make_finger(mirror: bool = False) -> cq.Workplane:
    """Build one finger in the SCHUNK STL coordinate frame."""
    half_width = FINGER_WIDTH / 2.0

    # Root plate + lower leg (one rectangle in X-Z), arm, and blade.
    # Root sits OUTSIDE the jaw face to clear the four guide rods.
    root_leg = box_xyz(
        JAW_MOUNT_FACE_X,
        ROOT_FRONT_X,
        -ROOT_WIDTH / 2.0,
        ROOT_WIDTH / 2.0,
        ROOT_LEG_BOTTOM_Z,
        ROOT_TOP_Z,
    )
    arm = box_xyz(
        BLADE_OUTER_X,
        JAW_MOUNT_FACE_X,
        -half_width,
        half_width,
        ARM_BOTTOM_Z,
        ARM_TOP_Z,
    )
    blade = box_xyz(
        BLADE_INNER_X,
        BLADE_OUTER_X,
        -half_width,
        half_width,
        TIP_Z,
        ARM_TOP_Z,
    )

    part = root_leg.union(arm).union(blade)

    # Stress-relief fillet at the blade/arm corner.
    part = part.edges(
        cq.selectors.NearestToPointSelector((BLADE_OUTER_X, 0.0, ARM_BOTTOM_Z))
    ).fillet(BLADE_FILLET_R)
    # Small gusset at the arm/root junction.  Keep it below the jaw's lower
    # edge so it cannot protrude into the base jaw.
    part = part.edges(
        cq.selectors.NearestToPointSelector((JAW_MOUNT_FACE_X, 0.0, ARM_TOP_Z))
    ).fillet(ARM_ROOT_FILLET_R)

    # Print-friendly chamfers: jaw-side perimeter and tip perimeter.
    if EDGE_CHAMFER > 0:
        part = part.faces(">X").edges().chamfer(EDGE_CHAMFER)
        part = part.faces("<Z").edges().chamfer(EDGE_CHAMFER)

    # Rubber-pad bonding recess on the grip face.
    pocket = box_xyz(
        BLADE_INNER_X,
        BLADE_INNER_X + PAD_THICKNESS,
        -PAD_WIDTH / 2.0,
        PAD_WIDTH / 2.0,
        PAD_Z0,
        PAD_Z1,
    )
    pocket = pocket.edges("|X").fillet(PAD_CORNER_R)
    part = part.cut(pocket)

    # Guide-rod relief pockets on the jaw-side face.
    for y, z, sy, sz in ROD_RELIEF_YZ_SIZE:
        relief = box_xyz(
            JAW_MOUNT_FACE_X - ROD_RELIEF_FACE_MARGIN,
            JAW_MOUNT_FACE_X + ROD_RELIEF_DEPTH,
            y - sy / 2.0,
            y + sy / 2.0,
            z - sz / 2.0,
            z + sz / 2.0,
        )
        part = part.cut(relief)

    # Mounting holes through the root + counterbores from the OUTER face.
    for y, z in MOUNT_HOLE_PATTERN_YZ:
        clearance = cq.Solid.makeCylinder(
            CLEARANCE_D / 2.0,
            ROOT_THICKNESS,
            cq.Vector(ROOT_FRONT_X, y, z),
            cq.Vector(-1, 0, 0),
        )
        cbore = cq.Solid.makeCylinder(
            CBORE_D / 2.0,
            MOUNT_CBORE_DEPTH,
            cq.Vector(ROOT_FRONT_X, y, z),
            cq.Vector(-1, 0, 0),
        )
        part = part.cut(cq.Workplane("XY").add(clearance))
        part = part.cut(cq.Workplane("XY").add(cbore))

    if mirror:
        part = part.mirror("YZ")

    solid = part.val()
    if not solid.isValid():
        raise RuntimeError("CadQuery produced an invalid finger solid")
    if len(part.solids().vals()) != 1:
        raise RuntimeError("Finger is not a single connected solid")
    return part


def print_orientation(part: cq.Workplane) -> cq.Workplane:
    """
    Rotate the part so its side face is on the bed:
        print X = X thickness/offset direction
        print Y = finger length (root at Y=0, tip at Y=FINGER_LENGTH)
        print Z = Y width
    The result sits at X>=0, Y>=0, Z>=0 with Z=0 on the bed.
    """
    p = part.val().rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90.0)
    bb = p.BoundingBox()
    p = p.translate(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin))
    return cq.Workplane("XY").add(p)


def bbox_dict(shape) -> dict:
    bb = shape.BoundingBox()
    return {
        "xmin": round(bb.xmin, 4),
        "xmax": round(bb.xmax, 4),
        "ymin": round(bb.ymin, 4),
        "ymax": round(bb.ymax, 4),
        "zmin": round(bb.zmin, 4),
        "zmax": round(bb.zmax, 4),
        "xlen": round(bb.xlen, 4),
        "ylen": round(bb.ylen, 4),
        "zlen": round(bb.zlen, 4),
    }


def main() -> None:
    validate_parameters()

    base = Path(__file__).resolve().parent
    stl_dir = base / "stl"
    step_dir = base / "step"
    urdf_dir = base / "urdf"
    for d in (stl_dir, step_dir, urdf_dir):
        d.mkdir(parents=True, exist_ok=True)

    right = make_finger(mirror=False)
    left = make_finger(mirror=True)

    # Assembly coordinates: exactly overlay the supplied SCHUNK STL.
    right_asm = right.val().translate(cq.Vector(0, JAW_CENTER_Y, 0))
    left_asm = left.val().translate(cq.Vector(0, JAW_CENTER_Y, 0))
    assembly = cq.Compound.makeCompound([right_asm, left_asm])

    right_print = print_orientation(right)
    left_print = print_orientation(left)

    # --- exports -------------------------------------------------------------
    cq.exporters.export(
        right_print,
        str(stl_dir / "finger_right_print.stl"),
        tolerance=STL_TOLERANCE,
        angularTolerance=STL_ANGULAR_TOLERANCE,
    )
    cq.exporters.export(
        left_print,
        str(stl_dir / "finger_left_print.stl"),
        tolerance=STL_TOLERANCE,
        angularTolerance=STL_ANGULAR_TOLERANCE,
    )
    cq.exporters.export(
        cq.Workplane("XY").add(assembly),
        str(stl_dir / "fingers_pair_assembly.stl"),
        tolerance=STL_TOLERANCE,
        angularTolerance=STL_ANGULAR_TOLERANCE,
    )
    cq.exporters.export(
        cq.Workplane("XY").add(right_asm),
        str(urdf_dir / "finger_right_assembly.stl"),
        tolerance=STL_TOLERANCE,
        angularTolerance=STL_ANGULAR_TOLERANCE,
    )
    cq.exporters.export(
        cq.Workplane("XY").add(left_asm),
        str(urdf_dir / "finger_left_assembly.stl"),
        tolerance=STL_TOLERANCE,
        angularTolerance=STL_ANGULAR_TOLERANCE,
    )
    cq.exporters.export(right, str(step_dir / "finger_right.step"))
    cq.exporters.export(left, str(step_dir / "finger_left.step"))
    cq.exporters.export(
        cq.Workplane("XY").add(assembly), str(step_dir / "fingers_pair_assembly.step")
    )

    # --- measurements --------------------------------------------------------
    right_vol_mm3 = right.val().Volume()
    left_vol_mm3 = left.val().Volume()
    assembly_vol_mm3 = right_vol_mm3 + left_vol_mm3
    right_com = cq.Shape.centerOfMass(right.val())
    right_mass_petg = right_vol_mm3 / 1000.0 * PETG_DENSITY_G_CM3
    right_mass_pa12 = right_vol_mm3 / 1000.0 * PA12CF_DENSITY_G_CM3
    pair_mass_petg = assembly_vol_mm3 / 1000.0 * PETG_DENSITY_G_CM3
    pair_mass_pa12 = assembly_vol_mm3 / 1000.0 * PA12CF_DENSITY_G_CM3

    grip_force_per_jaw = JAW_CLOSING_FORCE_TOTAL / 2.0
    lever_m = FINGER_LENGTH / 1000.0
    root_bending_moment = grip_force_per_jaw * lever_m
    # Full blade cross-section at the root (pocket starts lower down).
    section_modulus_mm3 = FINGER_WIDTH * BLADE_THICKNESS**2 / 6.0
    blade_root_stress_mpa = root_bending_moment * 1000.0 / section_modulus_mm3

    closed_finger_gap = JAW_GAP_CLOSED_AT_MOUNTS - 2.0 * GRIP_OFFSET_TOTAL
    open_finger_gap = (
        JAW_GAP_CLOSED_AT_MOUNTS + 2.0 * JAW_STROKE_PER_JAW
        - 2.0 * GRIP_OFFSET_TOTAL
    )
    closed_gap_with_pads = closed_finger_gap - 2.0 * PAD_THICKNESS
    open_gap_with_pads = open_finger_gap - 2.0 * PAD_THICKNESS
    com_inward_from_jaw = JAW_MOUNT_FACE_X - right_com.x
    com_drop_below_jaw_center = -right_com.z
    projection_from_jaw_top = JAW_FACE_Z_MAX - TIP_Z
    finger_weight_moment_about_jaw = (
        (right_mass_petg / 1000.0) * 9.81 * (com_inward_from_jaw / 1000.0)
    )

    report = {
        "units": "mm",
        "notes": [
            "STL carries no unit metadata; all geometry was modelled in millimetres.",
            "Assembly files use the supplied SCHUNK STL coordinate frame.",
            "Fingers are mounted on the OUTER jaw faces; confirm on the physical gripper.",
            "Guide-rod relief pockets are included so the root sits flat on the jaw face.",
            "GRIP_GAP_CLOSED and PAD_THICKNESS are placeholders pending user confirmation.",
        ],
        "design_parameters": {
            "FINGER_LENGTH": FINGER_LENGTH,
            "FINGER_WIDTH": FINGER_WIDTH,
            "ROOT_WIDTH": ROOT_WIDTH,
            "ROOT_HEIGHT": ROOT_HEIGHT,
            "ROOT_THICKNESS": ROOT_THICKNESS,
            "BLADE_THICKNESS": BLADE_THICKNESS,
            "ARM_HEIGHT": ARM_HEIGHT,
            "GRIP_GAP_CLOSED": GRIP_GAP_CLOSED,
            "GRIP_OFFSET_TOTAL": round(GRIP_OFFSET_TOTAL, 4),
            "PAD_THICKNESS": PAD_THICKNESS,
            "PAD_WIDTH": PAD_WIDTH,
            "PAD_LENGTH": PAD_LENGTH,
            "BLADE_FILLET_R": BLADE_FILLET_R,
            "ARM_ROOT_FILLET_R": ARM_ROOT_FILLET_R,
            "EDGE_CHAMFER": EDGE_CHAMFER,
            "ROD_RELIEF_YZ_SIZE": ROD_RELIEF_YZ_SIZE,
            "ROD_RELIEF_DEPTH": ROD_RELIEF_DEPTH,
            "MOUNT_THREAD": MOUNT_THREAD,
            "MOUNT_SCREW": MOUNT_SCREW,
            "MOUNT_HOLE_COMP": MOUNT_HOLE_COMP,
            "MOUNT_CLEARANCE_D": CLEARANCE_D,
            "MOUNT_CBORE_D": CBORE_D,
            "MOUNT_CBORE_DEPTH": MOUNT_CBORE_DEPTH,
            "MOUNT_HOLE_PATTERN_YZ": MOUNT_HOLE_PATTERN_YZ,
        },
        "measured_schunk_interface": {
            "jaw_mount_face_mm": [
                JAW_FACE_Y_MAX - JAW_FACE_Y_MIN,
                JAW_FACE_Z_MAX - JAW_FACE_Z_MIN,
            ],
            "jaw_face_y_range": [JAW_FACE_Y_MIN, JAW_FACE_Y_MAX],
            "jaw_face_z_range": [JAW_FACE_Z_MIN, JAW_FACE_Z_MAX],
            "jaw_face_center_y": JAW_CENTER_Y,
            "mount_side": "outer jaw face",
            "jaw_mount_gap_closed": JAW_GAP_CLOSED_AT_MOUNTS,
            "stroke_per_jaw": JAW_STROKE_PER_JAW,
            "guide_rod_protrusion_approx": 2.0,
        },
        "geometry": {
            "right_bbox_print": bbox_dict(right_print.val()),
            "left_bbox_print": bbox_dict(left_print.val()),
            "right_bbox_assembly": bbox_dict(right_asm),
            "left_bbox_assembly": bbox_dict(left_asm),
            "assembly_bbox": bbox_dict(assembly),
            "right_volume_cm3": round(right_vol_mm3 / 1000.0, 3),
            "left_volume_cm3": round(left_vol_mm3 / 1000.0, 3),
            "pair_volume_cm3": round(assembly_vol_mm3 / 1000.0, 3),
            "right_center_of_mass_assembly": [
                round(right_com.x, 3),
                round(right_com.y + JAW_CENTER_Y, 3),
                round(right_com.z, 3),
            ],
        },
        "mass_estimates_g": {
            "one_finger_petg": round(right_mass_petg, 1),
            "one_finger_pa12cf": round(right_mass_pa12, 1),
            "pair_petg": round(pair_mass_petg, 1),
            "pair_pa12cf": round(pair_mass_pa12, 1),
        },
        "screening_calculations": {
            "grip_force_per_jaw_N": grip_force_per_jaw,
            "root_bending_moment_Nm_at_tip": round(root_bending_moment, 2),
            "blade_section_modulus_mm3": round(section_modulus_mm3, 1),
            "blade_root_bending_stress_MPa": round(blade_root_stress_mpa, 1),
            "projection_from_jaw_top_mm": round(projection_from_jaw_top, 1),
            "closed_blade_gap_mm": round(closed_finger_gap, 1),
            "open_blade_gap_mm": round(open_finger_gap, 1),
            "closed_gap_with_2x_pads_mm": round(closed_gap_with_pads, 1),
            "open_gap_with_2x_pads_mm": round(open_gap_with_pads, 1),
            "com_inward_from_jaw_face_mm": round(com_inward_from_jaw, 1),
            "com_below_jaw_center_mm": round(com_drop_below_jaw_center, 1),
            "one_finger_weight_moment_about_jaw_Nm": round(
                finger_weight_moment_about_jaw, 3
            ),
        },
        "official_limits": {
            "max_finger_length_mm": MAX_PERMITTED_FINGER_LENGTH,
            "max_finger_weight_kg": MAX_PERMITTED_FINGER_WEIGHT,
            "Mx_Nm": MAX_MX,
            "My_Nm": MAX_MY,
            "Mz_Nm": MAX_MZ,
            "Fz_N": MAX_FZ,
            "nominal_pressure_bar": NOMINAL_PRESSURE_BAR,
        },
        "checks": {
            "finger_length_within_limit": FINGER_LENGTH <= MAX_PERMITTED_FINGER_LENGTH,
            "projection_from_jaw_top_within_limit": projection_from_jaw_top
            <= MAX_PERMITTED_FINGER_LENGTH,
            "finger_mass_within_limit_petg": right_mass_petg / 1000.0
            <= MAX_PERMITTED_FINGER_WEIGHT,
            "finger_mass_within_limit_pa12cf": right_mass_pa12 / 1000.0
            <= MAX_PERMITTED_FINGER_WEIGHT,
            "right_single_solid": len(right.solids().vals()) == 1,
            "left_single_solid": len(left.solids().vals()) == 1,
            "right_valid": right.val().isValid(),
            "left_valid": left.val().isValid(),
        },
    }

    report_path = base / "build_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("Built gripper fingers in:", base)
    print(f"  FINGER_LENGTH      : {FINGER_LENGTH:.1f} mm (limit {MAX_PERMITTED_FINGER_LENGTH:.0f} mm)")
    print(f"  projection from jaw top: {projection_from_jaw_top:.1f} mm (limit {MAX_PERMITTED_FINGER_LENGTH:.0f} mm)")
    print(f"  print bbox         : {bbox_dict(right_print.val())}")
    print(f"  one finger volume  : {right_vol_mm3/1000.0:.2f} cm^3")
    print(f"  one finger mass    : PETG {right_mass_petg:.1f} g | PA12-CF {right_mass_pa12:.1f} g")
    print(f"  pair mass          : PETG {pair_mass_petg:.1f} g | PA12-CF {pair_mass_pa12:.1f} g")
    print(f"  closed blade gap   : {closed_finger_gap:.1f} mm (with pads {closed_gap_with_pads:.1f} mm)")
    print(f"  open blade gap     : {open_finger_gap:.1f} mm (with pads {open_gap_with_pads:.1f} mm)")
    print(f"  blade root stress  : {blade_root_stress_mpa:.1f} MPa at 375 N tip load (screening only)")
    print("  report             :", report_path)


if __name__ == "__main__":
    main()
