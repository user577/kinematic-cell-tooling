"""
Kinematic Workholding Pallet System — Parametric FreeCAD Model
==============================================================

Design Rationale
----------------
Gravity retention fails for light pallets: any horizontal process force,
robot vibration, or accidental contact can tip or slide the pallet off a
gravity-only coupling. Magnetic preload helps but drops as the cube of
distance, offering near-zero lateral stiffness.

The correct separation of concerns:
  - KINEMATIC COUPLING handles XY + theta location (deterministic, repeatable).
  - TWIST-LOCK LATCH handles Z retention (positive mechanical interference).
  - Gravity + spring provides nesting preload (seating force only).

Axial twist-lock is superior to ball-race clamping (ATI, Schunk) because
location and retention are decoupled: the kinematic coupling seats without
friction or hysteresis, then the twist-lock engages independently. A $15
kinematic coupling outperforms a $2000 tool changer on repeatability because
the contact geometry is deterministic regardless of clamp force variation.

The cam-driven twist-lock is self-engaging on descent (no actuator to lock),
fail-safe on power loss (stays locked), and requires a solenoid only for
release. 45 degrees of rotation over 15mm Z travel locks tabs behind
receiver slot lips with positive mechanical interference.

Parametric Scaling
------------------
All geometry is driven from a PalletConfig object with three master inputs:
  - plate_size_mm: overall plate dimension (square)
  - ball_dia_mm: precision ball bearing diameter (from standard sizes)
  - bolt_size: mounting bolt (auto-selected or overridden)

Everything else is derived from these through geometric relationships and
proportional scaling. Presets are provided for common sizes:
  SMALL  =  80mm plate,  3mm balls, M4 bolts  (bench-top, light parts)
  MEDIUM = 120mm plate,  5mm balls, M6 bolts  (default, general robot cell)
  LARGE  = 200mm plate,  8mm balls, M8 bolts  (heavy pallets, machining)
  XL     = 300mm plate, 10mm balls, M10 bolts (large format, automotive)

Usage:
  FreeCAD GUI:  Macro → Run this script  (builds MEDIUM preset)
  Headless CLI:
    FreeCADCmd -c "exec(open('kinematic_pallet.py').read())"
    FreeCADCmd -c "PALLET_PLATE_SIZE=200; exec(open('kinematic_pallet.py').read())"
    FreeCADCmd -c "PALLET_PRESET='large'; exec(open('kinematic_pallet.py').read())"

Requires: FreeCAD 0.21+ with Part workbench.
"""

import math
import os
import sys

import FreeCAD
import Part

# ---------------------------------------------------------------------------
# Output paths — resolve relative to this script's location
# ---------------------------------------------------------------------------
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # Running via FreeCADCmd -c exec() — __file__ is not defined
    SCRIPT_DIR = os.path.abspath("C:/Users/user/kinematic-cell-tooling/cad/scripts")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir, os.pardir))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "cad", "outputs")
FREECAD_DIR = os.path.join(PROJECT_ROOT, "cad", "freecad")
os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(FREECAD_DIR, exist_ok=True)


# ===========================================================================
# STANDARD HARDWARE TABLES
# ===========================================================================
# These are the real-world hardware constraints. You don't scale a ball
# bearing — you pick from available sizes. Everything else adapts.

# Metric bolt specs: (clearance_hole_dia, counterbore_dia, counterbore_depth,
#                     nut_flat_width, thread_pitch)
BOLT_TABLE = {
    "M3":  (3.4,   6.5,  3.5,  5.5, 0.5),
    "M4":  (4.5,   8.0,  4.5,  7.0, 0.7),
    "M5":  (5.5,   9.5,  5.5,  8.0, 0.8),
    "M6":  (6.5,  11.0,  6.0, 10.0, 1.0),
    "M8":  (9.0,  14.5,  8.0, 13.0, 1.25),
    "M10": (11.0, 17.5, 10.0, 16.0, 1.5),
    "M12": (13.5, 20.5, 12.0, 18.0, 1.75),
}

# Standard precision ball bearing diameters (mm) — G25 chrome steel
STANDARD_BALL_SIZES = [3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 15.0]

# Standard ground dowel pin diameters (mm) — h6 tolerance
STANDARD_PIN_SIZES = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0]


# ===========================================================================
# PARAMETRIC CONFIGURATION
# ===========================================================================

class PalletConfig:
    """
    Parametric configuration for the kinematic pallet system.

    Three master inputs drive all geometry:
      plate_size_mm: overall plate dimension (square, both base and pallet)
      ball_dia_mm:   precision ball bearing diameter
      bolt_size:     mounting bolt size string (e.g. "M6")

    All other dimensions are derived from these through:
      - Geometric relationships (Hertzian contact, kinematic constraint)
      - Proportional scaling (plate features scale with plate size)
      - Hardware catalog constraints (nearest standard size)

    The derivation rules encode mechanical design intent:
      - Pin center distance is locked to ball+pin contact geometry
      - Bolt circle radius is ~29% of plate size (leaves room for mounting)
      - Base thickness is ~17% of plate size (stiffness vs. weight)
      - Receiver diameter is ~18% of plate size (latch engagement area)
    """

    def __init__(self, plate_size_mm=120.0, ball_dia_mm=5.0, bolt_size="M6",
                 label="custom"):
        self.label = label

        # --- Master inputs ---
        self.plate_size = float(plate_size_mm)
        self.ball_dia = float(ball_dia_mm)
        self.bolt_size = bolt_size

        # Scale factor relative to the 120mm reference design
        self.scale = self.plate_size / 120.0

        # --- Bolt hardware lookup ---
        if bolt_size not in BOLT_TABLE:
            raise ValueError(f"Unknown bolt size '{bolt_size}'. "
                             f"Choose from: {list(BOLT_TABLE.keys())}")
        bolt = BOLT_TABLE[bolt_size]
        self.mount_hole_dia = bolt[0]
        self.mount_cbore_dia = bolt[1]
        self.mount_cbore_depth = bolt[2]

        # --- Pin diameter: match ball diameter from standard sizes ---
        # Pin diameter should be <= ball diameter for proper cradle geometry.
        # Pick the largest standard pin that doesn't exceed the ball.
        self.pin_dia = max(p for p in STANDARD_PIN_SIZES if p <= self.ball_dia)

        # --- Kinematic coupling geometry (derived from physics) ---
        # Pin center-to-center in a two-pin cradle:
        #   distance = (ball_r + pin_r) * sqrt(2)
        # This places the ball tangent to both pins simultaneously.
        ball_r = self.ball_dia / 2.0
        pin_r = self.pin_dia / 2.0
        self.pin_center_dist = (ball_r + pin_r) * math.sqrt(2)

        # Bolt circle radius: ~29% of plate size.
        # Must leave clearance for mounting bolt pattern at corners.
        self.bolt_circle_r = round(self.plate_size * 0.292, 1)

        # --- Base station plate ---
        self.base_length = self.plate_size
        self.base_width = self.plate_size
        # Base thickness: ~17% of plate size, minimum 12mm for structural rigidity
        self.base_thick = max(12.0, round(self.plate_size * 0.167, 1))
        # Mounting bolt pattern: ~83% of plate size (corner-to-corner)
        self.mount_spacing = round(self.plate_size * 0.833, 1)

        # Pin protrusion and press depth scale with pin diameter
        self.pin_length = round(self.pin_dia * 3.2, 1)       # ~3.2x diameter
        self.pin_press_depth = round(self.pin_dia * 2.4, 1)  # ~2.4x diameter
        self.pin_protrusion = round(self.pin_length - self.pin_press_depth, 1)
        # Press-fit hole: 0.6% smaller than pin for interference fit
        self.pin_hole_dia = round(self.pin_dia * 0.994, 2)

        # --- Ball pocket geometry ---
        self.ball_pocket_dia = round(self.ball_dia * 0.994, 2)  # Light press
        self.ball_pocket_depth = round(self.ball_dia * 0.70, 1)  # 70% of ball buried
        self.ball_protrusion = round(self.ball_dia - self.ball_pocket_depth, 1)

        # --- Pallet plate ---
        self.pallet_length = self.plate_size
        self.pallet_width = self.plate_size
        # Pallet thickness: ~8% of plate size, minimum 6mm
        self.pallet_thick = max(6.0, round(self.plate_size * 0.083, 1))

        # Robot pick holes: one bolt size smaller than mounting bolts
        bolt_sizes = list(BOLT_TABLE.keys())
        mount_idx = bolt_sizes.index(bolt_size)
        pick_bolt = bolt_sizes[max(0, mount_idx - 1)]
        self.robot_hole_dia = BOLT_TABLE[pick_bolt][0]
        self.robot_hole_spacing = round(self.plate_size * 0.417, 1)

        # Workpiece mounting holes: same as robot pick bolt
        work_bolt = bolt_sizes[max(0, mount_idx - 2)]
        self.work_hole_dia = BOLT_TABLE[work_bolt][0]
        self.work_hole_spacing = round(self.plate_size * 0.667, 1)

        # --- Twist-lock receiver (base station center) ---
        # Receiver diameter: ~18% of plate size
        self.receiver_dia = round(self.plate_size * 0.183, 1)
        self.receiver_depth = round(self.base_thick * 0.40, 1)
        # Slot dimensions scale with receiver
        self.receiver_slot_width = round(self.receiver_dia * 0.18, 1)
        self.receiver_slot_depth = round(self.receiver_dia * 0.23, 1)
        self.receiver_slot_height = round(self.receiver_depth * 0.75, 1)
        self.num_receiver_slots = 4

        # --- Twist-lock pin (pallet center) ---
        # Latch pin diameter: receiver minus 2mm clearance (scaled)
        self.latch_pin_dia = round(self.receiver_dia - 2.0 * self.scale, 1)
        self.latch_pin_length = round(self.pallet_thick, 1)
        self.latch_tab_width = round(self.receiver_slot_width - 0.2 * self.scale, 1)
        self.latch_tab_height = round(self.receiver_slot_height * 0.80, 1)
        self.latch_tab_radial = round(self.receiver_slot_depth, 1)
        self.num_latch_tabs = 4
        self.latch_chamfer = round(max(1.0, 2.0 * self.scale), 1)
        self.latch_lock_angle = 45.0  # Always 45 degrees (geometric constant)

        # Internal bore for extraction tool: scales with latch pin
        bolt_for_bore = bolt_sizes[max(0, mount_idx)]
        self.latch_bore_dia = BOLT_TABLE[bolt_for_bore][0]

        # --- Solenoid pocket ---
        # Solenoid housing: ~21% of plate size, minimum 20mm
        self.solenoid_dia = max(20.0, round(self.plate_size * 0.208, 1))
        self.solenoid_depth = round(self.base_thick * 0.90, 1)
        # M3 mounting holes regardless of size (solenoid frame standard)
        self.solenoid_mount_hole_dia = BOLT_TABLE["M3"][0]
        self.solenoid_mount_offset = round(self.solenoid_dia * 0.64, 1)

        # --- Hall sensor pocket ---
        self.hall_pocket_dia = max(6.0, round(8.0 * self.scale, 1))
        self.hall_pocket_depth = round(self.hall_pocket_dia * 0.75, 1)
        self.hall_angle_deg = 60.0  # Always between cradle 1 and 2

        # --- Magnet pocket (pallet bottom face) ---
        self.magnet_pocket_dia = max(5.0, round(6.2 * self.scale, 1))
        self.magnet_pocket_depth = max(2.5, round(3.2 * self.scale, 1))

        # --- Wire channel ---
        self.wire_channel_width = max(2.0, round(3.0 * self.scale, 1))
        self.wire_channel_depth = max(1.5, round(2.0 * self.scale, 1))

        # --- Cam helix ---
        self.cam_z_travel = round(max(10.0, 15.0 * self.scale), 1)
        self.cam_rotation_deg = 45.0  # Always 45 degrees

        # --- Visualization offsets (scale with plate size) ---
        self.explode_gap = round(40.0 * self.scale, 0)
        self.descend_gap = round(15.0 * self.scale, 0)

        # --- Computed assembly values ---
        # Seated Z: base top face + pin protrusion - ball protrusion
        self.seated_z = self.base_thick + self.pin_protrusion - self.ball_protrusion
        self.face_gap = self.pin_protrusion - self.ball_protrusion

    def summary(self):
        """Print a human-readable summary of all key dimensions."""
        lines = [
            f"=== PalletConfig: {self.label} ===",
            f"  Plate size:       {self.plate_size:.0f} x {self.plate_size:.0f} mm",
            f"  Ball diameter:    {self.ball_dia:.1f} mm",
            f"  Pin diameter:     {self.pin_dia:.1f} mm",
            f"  Mounting bolts:   {self.bolt_size}",
            f"  Scale factor:     {self.scale:.3f}x (vs 120mm reference)",
            f"  ---",
            f"  Base thickness:   {self.base_thick:.1f} mm",
            f"  Pallet thickness: {self.pallet_thick:.1f} mm",
            f"  Bolt circle R:    {self.bolt_circle_r:.1f} mm",
            f"  Mount pattern:    {self.mount_spacing:.1f} mm",
            f"  Pin spacing:      {self.pin_center_dist:.2f} mm (cradle)",
            f"  Pin protrusion:   {self.pin_protrusion:.1f} mm",
            f"  Ball protrusion:  {self.ball_protrusion:.1f} mm",
            f"  Face gap seated:  {self.face_gap:.1f} mm",
            f"  Seated Z offset:  {self.seated_z:.1f} mm",
            f"  ---",
            f"  Receiver dia:     {self.receiver_dia:.1f} mm",
            f"  Latch pin dia:    {self.latch_pin_dia:.1f} mm",
            f"  Tab W x H x R:   {self.latch_tab_width:.1f} x "
            f"{self.latch_tab_height:.1f} x {self.latch_tab_radial:.1f} mm",
            f"  Cam travel:       {self.cam_z_travel:.1f} mm / {self.cam_rotation_deg:.0f} deg",
            f"  Solenoid pocket:  {self.solenoid_dia:.1f} mm dia",
        ]
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Preset configurations
# ---------------------------------------------------------------------------

PRESETS = {
    "small": PalletConfig(
        plate_size_mm=80, ball_dia_mm=3.0, bolt_size="M4", label="small"
    ),
    "medium": PalletConfig(
        plate_size_mm=120, ball_dia_mm=5.0, bolt_size="M6", label="medium"
    ),
    "large": PalletConfig(
        plate_size_mm=200, ball_dia_mm=8.0, bolt_size="M8", label="large"
    ),
    "xl": PalletConfig(
        plate_size_mm=300, ball_dia_mm=10.0, bolt_size="M10", label="xl"
    ),
}

# Colors (R, G, B as 0-1 floats) — not parameterized, purely cosmetic
COLOR_BASE = (0.70, 0.70, 0.75)
COLOR_PALLET = (0.80, 0.80, 0.85)
COLOR_DOWEL = (0.88, 0.88, 0.92)
COLOR_BALL = (0.92, 0.92, 0.96)
COLOR_LATCH = (0.60, 0.62, 0.65)
COLOR_SOLENOID = (0.20, 0.20, 0.20)


# ===========================================================================
# Utility functions
# ===========================================================================

def _vec(x, y, z):
    """Shorthand for FreeCAD.Vector."""
    return FreeCAD.Vector(x, y, z)


def _rotate_point_z(x, y, angle_deg):
    """Rotate a 2D point about the origin by angle_deg (CCW positive)."""
    rad = math.radians(angle_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)


def _make_cylinder(radius, height, pos=(0, 0, 0), direction=(0, 0, 1)):
    """Create a cylinder at pos along direction."""
    return Part.makeCylinder(radius, height, _vec(*pos), _vec(*direction))


def _make_box(length, width, height, center_xy=False, z_offset=0):
    """Create a box. If center_xy, center it on the XY origin."""
    box = Part.makeBox(length, width, height)
    if center_xy:
        box.translate(_vec(-length / 2.0, -width / 2.0, z_offset))
    else:
        box.translate(_vec(0, 0, z_offset))
    return box


def _translate_shape(shape, dx=0, dy=0, dz=0):
    """Return a translated copy of a shape."""
    copy = shape.copy()
    copy.translate(_vec(dx, dy, dz))
    return copy


def _rotate_shape_z(shape, angle_deg, center=(0, 0, 0)):
    """Return a copy of shape rotated about the Z axis."""
    copy = shape.copy()
    copy.rotate(_vec(*center), _vec(0, 0, 1), angle_deg)
    return copy


# ===========================================================================
# BASE STATION
# ===========================================================================

def make_base_station(cfg):
    """
    Build the base station: rectangular aluminum plate with three two-pin
    kinematic cradles, central twist-lock receiver, solenoid pocket,
    hall sensor pocket, and wire channels.

    The base station sits at Z=0 (bottom face) to Z=cfg.base_thick (top face).
    Coupling features are on the top face. Solenoid pocket is from the bottom.
    """

    # --- Main plate body ---
    # Rectangular plate centered on XY origin
    base = _make_box(cfg.base_length, cfg.base_width, cfg.base_thick, center_xy=True)

    # --- Corner mounting holes (counterbored, square bolt pattern) ---
    half_spacing = cfg.mount_spacing / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = sx * half_spacing
            cy = sy * half_spacing
            # Through hole for mounting bolt
            through = _make_cylinder(
                cfg.mount_hole_dia / 2.0, cfg.base_thick + 2,
                pos=(cx, cy, -1)
            )
            base = base.cut(through)
            # Counterbore from top face
            cbore = _make_cylinder(
                cfg.mount_cbore_dia / 2.0, cfg.mount_cbore_depth + 0.1,
                pos=(cx, cy, cfg.base_thick - cfg.mount_cbore_depth)
            )
            base = base.cut(cbore)

    # --- Three two-pin cradles at 120 degree spacing ---
    # Each cradle: two ground dowel pins oriented tangentially to the bolt
    # circle. "Tangential" means the line connecting pin centers is
    # perpendicular to the radial direction — this orientation maximizes
    # the coupling stiffness in all three constraint directions.
    pin_shapes = []
    cradle_spacing_deg = 120.0
    for i in range(3):
        cradle_angle = i * cradle_spacing_deg
        # Cradle center position on bolt circle
        cx, cy = _rotate_point_z(cfg.bolt_circle_r, 0, cradle_angle)

        # Tangential direction: 90 degrees CCW from radial
        tang_angle = cradle_angle + 90.0
        half_pin_sep = cfg.pin_center_dist / 2.0

        for sign in (-1, 1):
            # Pin center offset along tangential direction
            dx, dy = _rotate_point_z(half_pin_sep * sign, 0, tang_angle)
            px = cx + dx
            py = cy + dy

            # Press-fit hole from top face downward
            hole = _make_cylinder(
                cfg.pin_hole_dia / 2.0, cfg.pin_press_depth + 0.1,
                pos=(px, py, cfg.base_thick - cfg.pin_press_depth)
            )
            base = base.cut(hole)

            # Pin solid: full length, positioned so it protrudes above top face
            pin = _make_cylinder(
                cfg.pin_dia / 2.0, cfg.pin_length,
                pos=(px, py, cfg.base_thick - cfg.pin_press_depth)
            )
            pin_shapes.append(pin)

    # --- Central twist-lock receiver ---
    # Cylindrical recess from top face, centered on plate.
    # The receiver provides the female half of the twist-lock mechanism.
    receiver_recess = _make_cylinder(
        cfg.receiver_dia / 2.0, cfg.receiver_depth + 0.1,
        pos=(0, 0, cfg.base_thick - cfg.receiver_depth)
    )
    base = base.cut(receiver_recess)

    # Four latch receiver slots cut radially into the recess wall at 90 degrees.
    # The latch pin tabs slide through these slots during vertical entry,
    # then rotate behind the slot lips to create positive axial retention.
    for i in range(cfg.num_receiver_slots):
        slot_angle = i * (360.0 / cfg.num_receiver_slots)
        slot_box = Part.makeBox(
            cfg.receiver_slot_depth, cfg.receiver_slot_width, cfg.receiver_slot_height
        )
        # Center slot on the recess wall, extending radially outward
        slot_box.translate(_vec(
            -cfg.receiver_slot_depth / 2.0,
            -cfg.receiver_slot_width / 2.0,
            0
        ))
        # Move to recess wall radius
        slot_box.translate(_vec(
            cfg.receiver_dia / 2.0,
            0, 0
        ))
        # Z position: at bottom of recess
        slot_box.translate(_vec(0, 0, cfg.base_thick - cfg.receiver_depth))
        # Rotate to angular position
        slot_box.rotate(_vec(0, 0, 0), _vec(0, 0, 1), slot_angle)
        base = base.cut(slot_box)

    # --- Solenoid pocket from bottom face ---
    # Houses the rotary solenoid for twist-lock release.
    # Intentionally intersects receiver space — solenoid shaft passes through.
    solenoid_pocket = _make_cylinder(
        cfg.solenoid_dia / 2.0, min(cfg.solenoid_depth, cfg.base_thick - 2),
        pos=(0, 0, -0.1)
    )
    base = base.cut(solenoid_pocket)

    # M3 retention holes flanking solenoid pocket (through bottom face)
    for sx in (-1, 1):
        m3_hole = _make_cylinder(
            cfg.solenoid_mount_hole_dia / 2.0, cfg.base_thick + 2,
            pos=(sx * cfg.solenoid_mount_offset, 0, -1)
        )
        base = base.cut(m3_hole)

    # --- Hall effect sensor pocket ---
    # Between cradle positions 1 and 2, on the bolt circle.
    # Detects pallet seating via magnet in pallet bottom face.
    hall_x, hall_y = _rotate_point_z(cfg.bolt_circle_r, 0, cfg.hall_angle_deg)
    hall_pocket = _make_cylinder(
        cfg.hall_pocket_dia / 2.0, cfg.hall_pocket_depth + 0.1,
        pos=(hall_x, hall_y, cfg.base_thick - cfg.hall_pocket_depth)
    )
    base = base.cut(hall_pocket)

    # --- Wire channel network ---
    # Shallow slots on top face routing signals from hall sensor and solenoid
    # to a single exit point at the plate edge.
    channel_z = cfg.base_thick - cfg.wire_channel_depth

    # Segment 1: Hall pocket to plate center (solenoid area)
    seg1 = Part.makeBox(
        math.hypot(hall_x, hall_y) + 2,
        cfg.wire_channel_width,
        cfg.wire_channel_depth + 0.1
    )
    seg1.translate(_vec(0, -cfg.wire_channel_width / 2.0, channel_z))
    seg1.rotate(_vec(0, 0, 0), _vec(0, 0, 1), cfg.hall_angle_deg)
    base = base.cut(seg1)

    # Segment 2: Center to plate edge (exit point at -X edge)
    seg2 = Part.makeBox(
        cfg.base_length / 2.0 + 1,
        cfg.wire_channel_width,
        cfg.wire_channel_depth + 0.1
    )
    seg2.translate(_vec(
        -cfg.base_length / 2.0 - 0.5,
        -cfg.wire_channel_width / 2.0,
        channel_z
    ))
    base = base.cut(seg2)

    return base, pin_shapes


# ===========================================================================
# PALLET
# ===========================================================================

def make_pallet(cfg):
    """
    Build the pallet: lightweight aluminum plate with three ball pockets
    (bottom face), central twist-lock pin, robot pick holes (top face),
    workpiece mounting holes, and magnet pocket.

    Pallet local frame: bottom face at Z=0, top face at Z=cfg.pallet_thick.
    """

    # --- Main plate body ---
    pallet = _make_box(
        cfg.pallet_length, cfg.pallet_width, cfg.pallet_thick, center_xy=True
    )

    # --- Robot pick feature: 2x holes on center line ---
    for sx in (-1, 1):
        hole = _make_cylinder(
            cfg.robot_hole_dia / 2.0, cfg.pallet_thick + 2,
            pos=(sx * cfg.robot_hole_spacing / 2.0, 0, -1)
        )
        pallet = pallet.cut(hole)

    # --- Workpiece mounting: 4x holes, corner bolt pattern ---
    half_work = cfg.work_hole_spacing / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            hole = _make_cylinder(
                cfg.work_hole_dia / 2.0, cfg.pallet_thick + 2,
                pos=(sx * half_work, sy * half_work, -1)
            )
            pallet = pallet.cut(hole)

    # --- Three ball pockets on bottom face ---
    # Ball bearings nest into two-pin cradles on the base station.
    # Each ball-on-two-pins contact constrains 2 DOF; three cradles = 6 DOF.
    cradle_spacing_deg = 120.0
    ball_shapes = []
    for i in range(3):
        ball_angle = i * cradle_spacing_deg
        bx, by = _rotate_point_z(cfg.bolt_circle_r, 0, ball_angle)

        # Pocket: cylindrical bore from bottom face upward
        pocket = _make_cylinder(
            cfg.ball_pocket_dia / 2.0, cfg.ball_pocket_depth + 0.1,
            pos=(bx, by, -0.1)
        )
        pallet = pallet.cut(pocket)

        # Ball bearing solid (for visualization)
        # Ball center Z: it protrudes ball_protrusion below bottom face
        ball_center_z = -cfg.ball_protrusion + cfg.ball_dia / 2.0
        ball = Part.makeSphere(cfg.ball_dia / 2.0, _vec(bx, by, ball_center_z))
        ball_shapes.append(ball)

    # --- Central twist-lock pin protruding below pallet ---
    # This is the male half of the twist-lock. Four tabs on the pin engage
    # with slots in the base station receiver. A helical cam on the receiver
    # wall converts vertical descent into 45 degrees of rotation, locking
    # the tabs behind the slot lips.
    latch_pin = _make_cylinder(
        cfg.latch_pin_dia / 2.0, cfg.latch_pin_length,
        pos=(0, 0, -cfg.latch_pin_length),
    )

    # Chamfer at bottom of pin: guides pin into receiver recess
    chamfer_cone = Part.makeCone(
        cfg.latch_pin_dia / 2.0,
        cfg.latch_pin_dia / 2.0 - cfg.latch_chamfer,
        cfg.latch_chamfer,
        _vec(0, 0, -cfg.latch_pin_length),
    )
    chamfer_zone_cyl = _make_cylinder(
        cfg.latch_pin_dia / 2.0 + 0.1, cfg.latch_chamfer + 0.1,
        pos=(0, 0, -cfg.latch_pin_length - 0.05)
    )
    chamfer_cut = chamfer_zone_cyl.cut(chamfer_cone)
    latch_pin = latch_pin.cut(chamfer_cut)

    # Internal bore for extraction tool (smooth bore, thread not modeled)
    latch_pin = latch_pin.cut(_make_cylinder(
        cfg.latch_bore_dia / 2.0,
        cfg.latch_pin_length - cfg.latch_chamfer - 1,
        pos=(0, 0, -cfg.latch_pin_length + cfg.latch_chamfer)
    ))

    # Four latch tabs at 90 degree spacing on pin body
    for i in range(cfg.num_latch_tabs):
        tab_angle = i * (360.0 / cfg.num_latch_tabs)
        tab = Part.makeBox(
            cfg.latch_tab_width, cfg.latch_tab_radial, cfg.latch_tab_height
        )
        # Center on pin surface, extending radially outward
        tab.translate(_vec(
            -cfg.latch_tab_width / 2.0,
            0,
            -cfg.latch_tab_height
        ))
        # Move to pin outer radius
        tab.translate(_vec(0, cfg.latch_pin_dia / 2.0, 0))
        tab.rotate(_vec(0, 0, 0), _vec(0, 0, 1), tab_angle)
        latch_pin = latch_pin.fuse(tab)

    # --- Magnet pocket on bottom face ---
    # Aligns with hall sensor pocket on base station when seated.
    mag_x, mag_y = _rotate_point_z(cfg.bolt_circle_r, 0, cfg.hall_angle_deg)
    magnet_pocket = _make_cylinder(
        cfg.magnet_pocket_dia / 2.0, cfg.magnet_pocket_depth + 0.1,
        pos=(mag_x, mag_y, -0.1)
    )
    pallet = pallet.cut(magnet_pocket)

    return pallet, latch_pin, ball_shapes


# ===========================================================================
# HELICAL CAM SURFACE (reference geometry)
# ===========================================================================

def make_cam_reference(cfg):
    """
    Model the helical cam track on the receiver inner wall.

    The cam converts vertical descent into rotary locking motion, making
    the twist-lock self-engaging. Approximated as a series of small ruled
    segments (swept helix is unstable in Part workbench).
    """
    cam_slot_width = cfg.receiver_slot_width
    cam_inner_r = cfg.receiver_dia / 2.0 - 1.0 * cfg.scale
    cam_outer_r = cfg.receiver_dia / 2.0 + 1.0 * cfg.scale
    n_segments = 36

    z_top = cfg.base_thick
    cam_solids = []

    for seg in range(n_segments):
        t0 = seg / float(n_segments)
        t1 = (seg + 1) / float(n_segments)

        z0 = z_top - t0 * cfg.cam_z_travel
        z1 = z_top - t1 * cfg.cam_z_travel

        angle0 = t0 * cfg.cam_rotation_deg
        angle1 = t1 * cfg.cam_rotation_deg

        seg_height = abs(z1 - z0) + 0.1
        seg_box = Part.makeBox(
            cam_outer_r - cam_inner_r,
            cam_slot_width,
            seg_height
        )
        seg_box.translate(_vec(cam_inner_r, -cam_slot_width / 2.0, z1 - 0.05))
        mid_angle = (angle0 + angle1) / 2.0
        seg_box.rotate(_vec(0, 0, 0), _vec(0, 0, 1), mid_angle)
        cam_solids.append(seg_box)

    cam = cam_solids[0]
    for s in cam_solids[1:]:
        cam = cam.fuse(s)
    return cam


# ===========================================================================
# SOLENOID PLACEHOLDER
# ===========================================================================

def make_solenoid_placeholder(cfg):
    """Simple cylinder representing the rotary solenoid (visualization only)."""
    return _make_cylinder(
        cfg.solenoid_dia / 2.0 - 0.5 * cfg.scale,
        min(cfg.solenoid_depth - 1, cfg.base_thick - 3),
        pos=(0, 0, 1)
    )


# ===========================================================================
# DOCUMENT CONSTRUCTION
# ===========================================================================

def add_shape_to_doc(doc, name, shape, color=None):
    """Add a shape to the FreeCAD document as a Part::Feature and set color."""
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    if color is not None and hasattr(obj, "ViewObject") and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
    return obj


def _add_to_group(doc, group, name, shape, color):
    """Add a shape to doc inside a group, handling link adjustment."""
    obj = add_shape_to_doc(doc, name, shape, color)
    obj.adjustRelativeLinks(group)
    group.addObject(obj)
    return obj


def build_document(cfg=None):
    """
    Main entry point: builds the complete FreeCAD document with all components
    and four assembly visualization states.

    Args:
        cfg: PalletConfig instance. If None, uses MEDIUM preset.
    """
    if cfg is None:
        cfg = PRESETS["medium"]

    print(cfg.summary())

    # Create or reset the document
    doc_name = f"KinematicPallet_{cfg.label}"
    if FreeCAD.ActiveDocument and FreeCAD.ActiveDocument.Name == doc_name:
        FreeCAD.closeDocument(doc_name)
    doc = FreeCAD.newDocument(doc_name)

    # -----------------------------------------------------------------------
    # Build component shapes
    # -----------------------------------------------------------------------

    base_shape, pin_shapes = make_base_station(cfg)
    pallet_shape, latch_pin_shape, ball_shapes = make_pallet(cfg)
    cam_shape = make_cam_reference(cfg)
    solenoid_shape = make_solenoid_placeholder(cfg)

    # Fuse repeated elements for export
    pins_fused = pin_shapes[0]
    for p in pin_shapes[1:]:
        pins_fused = pins_fused.fuse(p)

    balls_fused = ball_shapes[0]
    for b in ball_shapes[1:]:
        balls_fused = balls_fused.fuse(b)

    pallet_with_pin = pallet_shape.fuse(latch_pin_shape)
    for b in ball_shapes:
        pallet_with_pin = pallet_with_pin.fuse(b)

    # -----------------------------------------------------------------------
    # Add individual components to document (reference shapes)
    # -----------------------------------------------------------------------

    comp = doc.addObject("App::DocumentObjectGroup", "COMPONENTS")
    _add_to_group(doc, comp, "BaseStation", base_shape, COLOR_BASE)
    _add_to_group(doc, comp, "DowelPins", pins_fused, COLOR_DOWEL)
    _add_to_group(doc, comp, "CamTrack", cam_shape, COLOR_LATCH)
    _add_to_group(doc, comp, "Solenoid", solenoid_shape, COLOR_SOLENOID)
    _add_to_group(doc, comp, "Pallet", pallet_shape, COLOR_PALLET)
    _add_to_group(doc, comp, "LatchPin", latch_pin_shape, COLOR_LATCH)
    _add_to_group(doc, comp, "BallBearings", balls_fused, COLOR_BALL)

    # -----------------------------------------------------------------------
    # Helper: add a full assembly state group
    # -----------------------------------------------------------------------
    def _add_state(group_name, pallet_dz, pallet_rot=0):
        """Create an assembly state group with base at Z=0 and pallet offset."""
        grp = doc.addObject("App::DocumentObjectGroup", group_name)
        prefix = group_name[:4]
        _add_to_group(doc, grp, f"{prefix}_Base", base_shape, COLOR_BASE)
        _add_to_group(doc, grp, f"{prefix}_Pins", pins_fused, COLOR_DOWEL)
        _add_to_group(doc, grp, f"{prefix}_Solenoid", solenoid_shape, COLOR_SOLENOID)

        p = _translate_shape(pallet_shape, dz=pallet_dz)
        lp = _translate_shape(latch_pin_shape, dz=pallet_dz)
        bl = _translate_shape(balls_fused, dz=pallet_dz)
        if pallet_rot != 0:
            p = _rotate_shape_z(p, pallet_rot)
            lp = _rotate_shape_z(lp, pallet_rot)
            bl = _rotate_shape_z(bl, pallet_rot)

        _add_to_group(doc, grp, f"{prefix}_Pallet", p, COLOR_PALLET)
        _add_to_group(doc, grp, f"{prefix}_LatchPin", lp, COLOR_LATCH)
        _add_to_group(doc, grp, f"{prefix}_Balls", bl, COLOR_BALL)
        return grp

    # STATE 1: COMPONENTS_EXPLODED — all parts separated in Z
    _add_state("COMPONENTS_EXPLODED",
               pallet_dz=cfg.seated_z + cfg.explode_gap)

    # STATE 2: DESCENDING — pallet approaching, tabs aligned with slots
    _add_state("DESCENDING",
               pallet_dz=cfg.seated_z + cfg.descend_gap)

    # STATE 3: SEATED_UNLOCKED — balls in cradles, tabs aligned (not rotated)
    _add_state("SEATED_UNLOCKED",
               pallet_dz=cfg.seated_z)

    # STATE 4: SEATED_LOCKED — fully engaged, tabs rotated 45 degrees
    # The kinematic coupling balls-on-pins accommodate the small rotation
    # because each ball can rotate freely in its two-pin cradle.
    _add_state("SEATED_LOCKED",
               pallet_dz=cfg.seated_z, pallet_rot=cfg.latch_lock_angle)

    # -----------------------------------------------------------------------
    # Recompute
    # -----------------------------------------------------------------------
    doc.recompute()

    # -----------------------------------------------------------------------
    # Export STL via Mesh module (works in headless FreeCADCmd)
    # -----------------------------------------------------------------------
    import Mesh
    import MeshPart

    # Scale mesh quality with part size
    lin_defl = max(0.05, 0.1 * cfg.scale)
    ang_defl = 0.5

    def _export_stl(shape, filename):
        filepath = os.path.join(OUTPUT_DIR, filename)
        mesh = MeshPart.meshFromShape(
            Shape=shape,
            LinearDeflection=lin_defl,
            AngularDeflection=ang_defl,
        )
        mesh.write(filepath)
        print(f"  Exported: {filename}  ({mesh.CountFacets} facets)")

    label = cfg.label
    print(f"\nExporting STL ({label}):")

    # Base station with pins (single manufacturing piece + pressed pins)
    base_with_pins = base_shape.fuse(pins_fused)
    _export_stl(base_with_pins, f"base_station_{label}.stl")

    # Pallet with latch pin and balls
    _export_stl(pallet_with_pin, f"pallet_{label}.stl")

    # Latch pin alone (for detail views / printing)
    _export_stl(latch_pin_shape, f"latch_pin_{label}.stl")

    # Full locked assembly (merged visualization)
    locked_assy = base_with_pins.fuse(solenoid_shape)
    seated_locked = _translate_shape(pallet_with_pin, dz=cfg.seated_z)
    seated_locked = _rotate_shape_z(seated_locked, cfg.latch_lock_angle)
    locked_assy = locked_assy.fuse(seated_locked)
    _export_stl(locked_assy, f"assembly_locked_{label}.stl")

    # -----------------------------------------------------------------------
    # Save FreeCAD document
    # -----------------------------------------------------------------------
    fcstd_path = os.path.join(FREECAD_DIR, f"kinematic_pallet_{label}.FCStd")
    doc.saveAs(fcstd_path)
    print(f"  Saved: {fcstd_path}")

    print(f"\n=== Build complete: {label} ({cfg.plate_size:.0f}mm) ===")
    return doc


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == "__main__":
    # Check for global overrides (set before exec() in FreeCADCmd -c mode)
    preset_name = globals().get("PALLET_PRESET", None)
    plate_size = globals().get("PALLET_PLATE_SIZE", None)
    ball_dia = globals().get("PALLET_BALL_DIA", None)
    bolt_size = globals().get("PALLET_BOLT_SIZE", None)
    build_all = globals().get("PALLET_BUILD_ALL", False)

    if build_all:
        # Build all presets for comparison
        for name, preset_cfg in PRESETS.items():
            print(f"\n{'='*60}")
            build_document(preset_cfg)
    elif preset_name:
        # Use a named preset
        if preset_name not in PRESETS:
            raise ValueError(f"Unknown preset '{preset_name}'. "
                             f"Choose from: {list(PRESETS.keys())}")
        build_document(PRESETS[preset_name])
    elif plate_size:
        # Custom size from globals
        cfg = PalletConfig(
            plate_size_mm=plate_size,
            ball_dia_mm=ball_dia or 5.0,
            bolt_size=bolt_size or "M6",
            label=f"{int(plate_size)}mm",
        )
        build_document(cfg)
    else:
        # Default: medium preset
        build_document(PRESETS["medium"])
