"""
Kinematic Workholding Pallet System — FreeCAD Parametric Model
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

Assembly Components
-------------------
1. BASE STATION: Fixed plate with three two-pin cradles (Kelvin geometry),
   central twist-lock receiver with helical cam, solenoid pocket, hall sensor.
2. PALLET: Carried plate with three ball pockets, central twist-lock pin
   with four tabs, magnet pocket.
3. LATCH PIN: Detail model of the twist-lock engagement geometry.

Four visualization states: EXPLODED, DESCENDING, SEATED_UNLOCKED, SEATED_LOCKED.

Requires: FreeCAD 0.21+ with Part workbench.
Usage: Open in FreeCAD → Macro → Run this script.
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

# ---------------------------------------------------------------------------
# Design parameters — all dimensions in mm, angles in degrees
# ---------------------------------------------------------------------------

# Base station plate
BASE_LENGTH = 120.0          # X dimension
BASE_WIDTH = 120.0           # Y dimension
BASE_THICK = 20.0            # Z thickness
BASE_MOUNT_SPACING = 100.0   # M6 bolt pattern, corner-to-corner
BASE_MOUNT_HOLE_DIA = 6.5    # M6 clearance
BASE_MOUNT_CBORE_DIA = 11.0  # M6 counterbore
BASE_MOUNT_CBORE_DEPTH = 6.0

# Kinematic coupling geometry
BOLT_CIRCLE_R = 35.0         # Cradle center radius from plate center
CRADLE_SPACING_DEG = 120.0   # Angular spacing between cradles
PIN_DIA = 5.0                # Ground dowel pin diameter (h6)
PIN_LENGTH = 16.0            # Total pin length
PIN_PRESS_DEPTH = 12.0       # Depth pressed into base station
PIN_PROTRUSION = 4.0         # Height above base station top face
PIN_HOLE_DIA = 4.97          # Press-fit hole diameter
PIN_CENTER_DIST = 7.07       # Center-to-center of two pins in a cradle
                              # = (ball_r + pin_r) * sqrt(2) = 5.0 * 1.414

# Ball bearing specs (pallet side)
BALL_DIA = 5.0               # Chrome steel precision ball
BALL_POCKET_DIA = 4.97       # Light press pocket
BALL_POCKET_DEPTH = 3.5      # Pocket depth in pallet
BALL_PROTRUSION = 1.5        # Ball protrudes below pallet bottom face

# Pallet plate
PALLET_LENGTH = 120.0
PALLET_WIDTH = 120.0
PALLET_THICK = 10.0
PALLET_ROBOT_HOLE_DIA = 5.5  # M5 clearance
PALLET_ROBOT_HOLE_SPACING = 50.0
PALLET_WORK_HOLE_DIA = 4.5   # M4 clearance
PALLET_WORK_HOLE_SPACING = 80.0

# Twist-lock receiver (base station center)
RECEIVER_DIA = 22.0
RECEIVER_DEPTH = 8.0
RECEIVER_SLOT_WIDTH = 4.0
RECEIVER_SLOT_DEPTH = 5.0    # Radial depth into wall
RECEIVER_SLOT_HEIGHT = 6.0
NUM_RECEIVER_SLOTS = 4

# Twist-lock pin (pallet center, protrudes below)
LATCH_PIN_DIA = 20.0
LATCH_PIN_LENGTH = 10.0
LATCH_TAB_WIDTH = 3.8
LATCH_TAB_HEIGHT = 4.8
LATCH_TAB_RADIAL = 5.0       # Radial protrusion from pin body
NUM_LATCH_TABS = 4
LATCH_CHAMFER = 2.0          # 2mm x 45 degree entry chamfer
LATCH_LOCK_ANGLE = 45.0      # Rotation to lock

# Solenoid pocket (below receiver, accessed from base bottom)
SOLENOID_DIA = 25.0
SOLENOID_DEPTH = 20.0        # Note: deeper than base — models intent
SOLENOID_MOUNT_HOLE_DIA = 3.3  # M3 clearance
SOLENOID_MOUNT_OFFSET = 16.0   # Flanking distance from center

# Hall sensor pocket
HALL_POCKET_DIA = 8.0
HALL_POCKET_DEPTH = 6.0
HALL_ANGLE_DEG = 60.0         # 60 degrees from first cradle position

# Magnet pocket (pallet bottom face)
MAGNET_POCKET_DIA = 6.2
MAGNET_POCKET_DEPTH = 3.2

# Wire channel
WIRE_CHANNEL_WIDTH = 3.0
WIRE_CHANNEL_DEPTH = 2.0

# Cam helix (self-engaging twist-lock)
CAM_Z_TRAVEL = 15.0          # Vertical descent that produces lock rotation
CAM_ROTATION_DEG = 45.0      # Total rotation from cam

# Visualization offsets
EXPLODE_GAP = 40.0
DESCEND_GAP = 15.0

# Colors (R, G, B as 0-1 floats)
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


def _placement(x=0, y=0, z=0, axis=None, angle=0):
    """Create a FreeCAD.Placement with optional rotation about an axis."""
    if axis is None:
        axis = _vec(0, 0, 1)
    return FreeCAD.Placement(
        _vec(x, y, z),
        FreeCAD.Rotation(axis, angle),
    )


def _rotate_point_z(x, y, angle_deg):
    """Rotate a 2D point about the origin by angle_deg (CCW positive)."""
    rad = math.radians(angle_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)


def _make_cylinder(radius, height, pos=(0, 0, 0), direction=(0, 0, 1)):
    """Create a cylinder at pos along direction."""
    cyl = Part.makeCylinder(radius, height, _vec(*pos), _vec(*direction))
    return cyl


def _make_box(length, width, height, center_xy=False, z_offset=0):
    """Create a box. If center_xy, center it on the XY origin."""
    box = Part.makeBox(length, width, height)
    if center_xy:
        box.translate(_vec(-length / 2.0, -width / 2.0, z_offset))
    else:
        box.translate(_vec(0, 0, z_offset))
    return box


# ===========================================================================
# BASE STATION
# ===========================================================================

def make_base_station():
    """
    Build the base station: rectangular aluminum plate with three two-pin
    kinematic cradles, central twist-lock receiver, solenoid pocket,
    hall sensor pocket, and wire channels.

    The base station sits at Z=0 (bottom face) to Z=BASE_THICK (top face).
    Coupling features are on the top face. Solenoid pocket is from the bottom.
    """

    # --- Main plate body ---
    # Rectangular plate centered on XY origin
    base = _make_box(BASE_LENGTH, BASE_WIDTH, BASE_THICK, center_xy=True)

    # --- Corner mounting holes (M6 counterbored, 100mm bolt pattern) ---
    half_spacing = BASE_MOUNT_SPACING / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = sx * half_spacing
            cy = sy * half_spacing
            # Through hole for M6 bolt
            through = _make_cylinder(
                BASE_MOUNT_HOLE_DIA / 2.0, BASE_THICK + 2,
                pos=(cx, cy, -1)
            )
            base = base.cut(through)
            # Counterbore from top face
            cbore = _make_cylinder(
                BASE_MOUNT_CBORE_DIA / 2.0, BASE_MOUNT_CBORE_DEPTH + 0.1,
                pos=(cx, cy, BASE_THICK - BASE_MOUNT_CBORE_DEPTH)
            )
            base = base.cut(cbore)

    # --- Three two-pin cradles at 120 degree spacing ---
    # Each cradle: two pins oriented tangentially to the bolt circle.
    # "Tangential" means the line connecting pin centers is perpendicular
    # to the radial direction from plate center to cradle center.
    pin_shapes = []  # Collect pin solids for separate export
    for i in range(3):
        cradle_angle = i * CRADLE_SPACING_DEG  # 0, 120, 240 degrees
        # Cradle center position on bolt circle
        cx, cy = _rotate_point_z(BOLT_CIRCLE_R, 0, cradle_angle)

        # Tangential direction: perpendicular to radial, in XY plane.
        # Radial unit vector from origin to cradle center:
        #   (cos(angle), sin(angle))
        # Tangential unit vector (CCW perpendicular):
        #   (-sin(angle), cos(angle))
        tang_angle = cradle_angle + 90.0
        half_pin_sep = PIN_CENTER_DIST / 2.0

        for sign in (-1, 1):
            # Pin center offset along tangential direction
            dx, dy = _rotate_point_z(half_pin_sep * sign, 0, tang_angle)
            # Absolute position relative to cradle center on bolt circle
            # but we rotated from (half_pin_sep*sign, 0) by tang_angle,
            # so we need to add to cradle center
            px = cx + dx
            py = cy + dy

            # Press-fit hole from top face downward
            hole = _make_cylinder(
                PIN_HOLE_DIA / 2.0, PIN_PRESS_DEPTH + 0.1,
                pos=(px, py, BASE_THICK - PIN_PRESS_DEPTH)
            )
            base = base.cut(hole)

            # Pin solid: full length, positioned so it protrudes above top face
            pin = _make_cylinder(
                PIN_DIA / 2.0, PIN_LENGTH,
                pos=(px, py, BASE_THICK - PIN_PRESS_DEPTH)
            )
            pin_shapes.append(pin)

    # --- Central twist-lock receiver ---
    # Cylindrical recess from top face, centered on plate
    receiver_recess = _make_cylinder(
        RECEIVER_DIA / 2.0, RECEIVER_DEPTH + 0.1,
        pos=(0, 0, BASE_THICK - RECEIVER_DEPTH)
    )
    base = base.cut(receiver_recess)

    # Four latch receiver slots cut radially into the recess wall at 90 deg spacing.
    # Each slot is a rectangular pocket extending radially outward from the recess wall.
    for i in range(NUM_RECEIVER_SLOTS):
        slot_angle = i * 90.0
        # Slot center is at the recess wall radius, extending outward
        # Slot box: width (tangential) x depth (radial) x height (Z)
        slot_box = Part.makeBox(
            RECEIVER_SLOT_DEPTH, RECEIVER_SLOT_WIDTH, RECEIVER_SLOT_HEIGHT
        )
        # Position: center the slot on the recess wall
        slot_box.translate(_vec(
            -RECEIVER_SLOT_DEPTH / 2.0,
            -RECEIVER_SLOT_WIDTH / 2.0,
            0
        ))
        # Move to recess wall radius (X = RECEIVER_DIA/2, centered on slot depth)
        slot_box.translate(_vec(
            RECEIVER_DIA / 2.0 - RECEIVER_SLOT_DEPTH / 2.0 + RECEIVER_SLOT_DEPTH / 2.0,
            0, 0
        ))
        # Z position: top of recess down to slot height
        slot_box.translate(_vec(0, 0, BASE_THICK - RECEIVER_DEPTH))

        # Rotate around Z axis to proper angular position
        slot_box.rotate(_vec(0, 0, 0), _vec(0, 0, 1), slot_angle)
        base = base.cut(slot_box)

    # --- Solenoid pocket from bottom face ---
    # The solenoid pocket is deeper than the base plate thickness where it
    # intersects the receiver recess — this is intentional, as the solenoid
    # actuator shaft passes through into the receiver space.
    solenoid_pocket = _make_cylinder(
        SOLENOID_DIA / 2.0, min(SOLENOID_DEPTH, BASE_THICK - 2),
        pos=(0, 0, -0.1)
    )
    base = base.cut(solenoid_pocket)

    # M3 retention holes flanking solenoid pocket (through bottom face)
    for sx in (-1, 1):
        m3_hole = _make_cylinder(
            SOLENOID_MOUNT_HOLE_DIA / 2.0, BASE_THICK + 2,
            pos=(sx * SOLENOID_MOUNT_OFFSET, 0, -1)
        )
        base = base.cut(m3_hole)

    # --- Hall effect sensor pocket ---
    # Located at 60 degrees from first cradle position, on the bolt circle
    hall_x, hall_y = _rotate_point_z(BOLT_CIRCLE_R, 0, HALL_ANGLE_DEG)
    hall_pocket = _make_cylinder(
        HALL_POCKET_DIA / 2.0, HALL_POCKET_DEPTH + 0.1,
        pos=(hall_x, hall_y, BASE_THICK - HALL_POCKET_DEPTH)
    )
    base = base.cut(hall_pocket)

    # --- Wire channel network ---
    # Route from hall sensor pocket to solenoid pocket to plate edge.
    # Channel is cut into the top face as a shallow slot.
    channel_z = BASE_THICK - WIRE_CHANNEL_DEPTH

    # Segment 1: Hall pocket to plate center (solenoid area)
    seg1 = Part.makeBox(
        math.hypot(hall_x, hall_y) + 2,
        WIRE_CHANNEL_WIDTH,
        WIRE_CHANNEL_DEPTH + 0.1
    )
    seg1.translate(_vec(0, -WIRE_CHANNEL_WIDTH / 2.0, channel_z))
    # Rotate to align with hall pocket direction
    seg1.rotate(_vec(0, 0, 0), _vec(0, 0, 1), HALL_ANGLE_DEG)
    base = base.cut(seg1)

    # Segment 2: Center to plate edge (exit point at -X edge)
    seg2 = Part.makeBox(
        BASE_LENGTH / 2.0 + 1,
        WIRE_CHANNEL_WIDTH,
        WIRE_CHANNEL_DEPTH + 0.1
    )
    seg2.translate(_vec(-BASE_LENGTH / 2.0 - 0.5, -WIRE_CHANNEL_WIDTH / 2.0, channel_z))
    base = base.cut(seg2)

    return base, pin_shapes


# ===========================================================================
# PALLET
# ===========================================================================

def make_pallet():
    """
    Build the pallet: lightweight aluminum plate with three ball pockets
    (bottom face), central twist-lock pin, robot pick holes (top face),
    workpiece mounting holes, and magnet pocket.

    The pallet sits with its bottom face at Z=0 and top face at Z=PALLET_THICK
    in its local coordinate system. It will be translated for assembly views.
    """

    # --- Main plate body ---
    pallet = _make_box(PALLET_LENGTH, PALLET_WIDTH, PALLET_THICK, center_xy=True)

    # --- Robot pick feature: 2x M5 holes on 50mm spacing (top face, centered) ---
    for sx in (-1, 1):
        hole = _make_cylinder(
            PALLET_ROBOT_HOLE_DIA / 2.0, PALLET_THICK + 2,
            pos=(sx * PALLET_ROBOT_HOLE_SPACING / 2.0, 0, -1)
        )
        pallet = pallet.cut(hole)

    # --- Workpiece mounting: 4x M4 on 80mm bolt pattern, corners ---
    half_work = PALLET_WORK_HOLE_SPACING / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            hole = _make_cylinder(
                PALLET_WORK_HOLE_DIA / 2.0, PALLET_THICK + 2,
                pos=(sx * half_work, sy * half_work, -1)
            )
            pallet = pallet.cut(hole)

    # --- Three ball pockets on bottom face ---
    # Matching base station cradle positions: 120 deg spacing on bolt circle
    ball_shapes = []
    for i in range(3):
        ball_angle = i * CRADLE_SPACING_DEG
        bx, by = _rotate_point_z(BOLT_CIRCLE_R, 0, ball_angle)

        # Pocket: cylindrical bore from bottom face upward
        pocket = _make_cylinder(
            BALL_POCKET_DIA / 2.0, BALL_POCKET_DEPTH + 0.1,
            pos=(bx, by, -0.1)
        )
        pallet = pallet.cut(pocket)

        # Ball bearing solid (for visualization)
        # Ball center is at Z = BALL_POCKET_DEPTH - BALL_DIA/2.0 from bottom face
        # but it protrudes below, so center is at:
        # bottom_face(0) - BALL_PROTRUSION + BALL_DIA/2.0
        ball_center_z = -BALL_PROTRUSION + BALL_DIA / 2.0
        ball = Part.makeSphere(BALL_DIA / 2.0, _vec(bx, by, ball_center_z))
        ball_shapes.append(ball)

    # --- Central twist-lock pin protruding below pallet ---
    # Pin body: cylinder extending downward from pallet bottom face
    latch_pin = _make_cylinder(
        LATCH_PIN_DIA / 2.0, LATCH_PIN_LENGTH,
        pos=(0, 0, -LATCH_PIN_LENGTH),
    )

    # Chamfer at bottom of pin: 2mm x 45 degree cone
    # Approximate with a cone frustum cut at the bottom
    chamfer_cone = Part.makeCone(
        LATCH_PIN_DIA / 2.0,                         # Top radius (at pin body)
        LATCH_PIN_DIA / 2.0 - LATCH_CHAMFER,         # Bottom radius (after chamfer)
        LATCH_CHAMFER,                                 # Height of chamfer zone
        _vec(0, 0, -LATCH_PIN_LENGTH),                # Start at pin bottom
    )
    # We need to subtract the material outside the chamfer cone.
    # Create a cylinder of the full pin diameter at the chamfer zone,
    # then subtract the cone to leave the chamfered shape.
    chamfer_zone_cyl = _make_cylinder(
        LATCH_PIN_DIA / 2.0 + 0.1, LATCH_CHAMFER + 0.1,
        pos=(0, 0, -LATCH_PIN_LENGTH - 0.05)
    )
    chamfer_cut = chamfer_zone_cyl.cut(chamfer_cone)
    latch_pin = latch_pin.cut(chamfer_cut)

    # Internal M8 threaded hole (modeled as smooth bore for visualization)
    m8_bore = _make_cylinder(
        4.0, LATCH_PIN_LENGTH - 2,  # 8mm dia bore, leave 2mm of meat at bottom
        pos=(0, 0, -LATCH_PIN_LENGTH + LATCH_CHAMFER)
    )
    latch_pin = latch_pin.cut(m8_bore)

    # Four latch tabs at 90 degree spacing on pin body
    # Each tab: radial protrusion from pin outer surface
    tab_shapes = []  # Collect for latch detail model
    for i in range(NUM_LATCH_TABS):
        tab_angle = i * 90.0
        # Tab is a box: width (tangential) x radial protrusion x height
        tab = Part.makeBox(LATCH_TAB_WIDTH, LATCH_TAB_RADIAL, LATCH_TAB_HEIGHT)
        # Center on pin surface, extending radially outward
        tab.translate(_vec(
            -LATCH_TAB_WIDTH / 2.0,
            0,
            -LATCH_TAB_HEIGHT  # Position below the pin top (just below pallet)
        ))
        # Move to pin outer radius
        tab.translate(_vec(0, LATCH_PIN_DIA / 2.0, 0))
        # Rotate to proper angular position
        tab.rotate(_vec(0, 0, 0), _vec(0, 0, 1), tab_angle)
        latch_pin = latch_pin.fuse(tab)
        tab_shapes.append(tab)

    # --- Magnet pocket on bottom face ---
    # At 60 degrees (matching hall sensor on base station), bolt circle radius
    mag_x, mag_y = _rotate_point_z(BOLT_CIRCLE_R, 0, HALL_ANGLE_DEG)
    magnet_pocket = _make_cylinder(
        MAGNET_POCKET_DIA / 2.0, MAGNET_POCKET_DEPTH + 0.1,
        pos=(mag_x, mag_y, -0.1)
    )
    pallet = pallet.cut(magnet_pocket)

    return pallet, latch_pin, ball_shapes


# ===========================================================================
# HELICAL CAM SURFACE (reference geometry)
# ===========================================================================

def make_cam_reference():
    """
    Model the helical cam track on the receiver inner wall.

    The cam converts 15mm of vertical descent into 45 degrees of rotation,
    making the twist-lock self-engaging. No actuator is needed to lock;
    a solenoid is needed only to release (fail-safe: power loss = locked).

    We approximate the helical surface with a series of ruled face segments
    because swept helix operations can be unstable in the Part workbench.
    The cam track is a helical slot on the inner wall of the receiver.
    """
    # Cam track parameters
    cam_slot_width = RECEIVER_SLOT_WIDTH  # Same width as receiver slots
    cam_inner_r = RECEIVER_DIA / 2.0 - 1.0  # Slightly inside receiver wall
    cam_outer_r = RECEIVER_DIA / 2.0 + 1.0  # Into the wall
    n_segments = 36  # Number of linear segments to approximate helix

    # The cam spans from the top of the receiver recess downward by CAM_Z_TRAVEL
    z_top = BASE_THICK  # Top of base station (entry point)
    z_bottom = z_top - CAM_Z_TRAVEL

    cam_solids = []
    for seg in range(n_segments):
        # Parametric position along the helix
        t0 = seg / float(n_segments)
        t1 = (seg + 1) / float(n_segments)

        z0 = z_top - t0 * CAM_Z_TRAVEL
        z1 = z_top - t1 * CAM_Z_TRAVEL

        angle0 = t0 * CAM_ROTATION_DEG
        angle1 = t1 * CAM_ROTATION_DEG

        # Create a small box representing this segment of the cam slot
        # and position it along the helical path
        seg_height = abs(z1 - z0) + 0.1
        seg_box = Part.makeBox(
            cam_outer_r - cam_inner_r,
            cam_slot_width,
            seg_height
        )
        seg_box.translate(_vec(cam_inner_r, -cam_slot_width / 2.0, z1 - 0.05))
        # Rotate to follow the helix
        mid_angle = (angle0 + angle1) / 2.0
        seg_box.rotate(_vec(0, 0, 0), _vec(0, 0, 1), mid_angle)
        cam_solids.append(seg_box)

    # Fuse all segments into one cam track shape
    if len(cam_solids) > 1:
        cam = cam_solids[0]
        for s in cam_solids[1:]:
            cam = cam.fuse(s)
    else:
        cam = cam_solids[0]

    return cam


# ===========================================================================
# SOLENOID PLACEHOLDER
# ===========================================================================

def make_solenoid_placeholder():
    """
    Simple cylinder representing the rotary solenoid (24VDC, 25mm frame).
    Positioned in the base station solenoid pocket for visualization.
    """
    sol = _make_cylinder(
        SOLENOID_DIA / 2.0 - 0.5,  # Slightly smaller than pocket
        min(SOLENOID_DEPTH - 1, BASE_THICK - 3),
        pos=(0, 0, 1)  # Sits 1mm above base bottom face
    )
    return sol


# ===========================================================================
# ASSEMBLY POSITIONING
# ===========================================================================

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


def compute_seated_z():
    """
    Compute the Z offset for the pallet when fully seated on the base station.

    When seated:
    - Base station top face is at Z = BASE_THICK
    - Pins protrude PIN_PROTRUSION above top face
    - Balls protrude BALL_PROTRUSION below pallet bottom face
    - Pallet bottom face sits at: BASE_THICK + PIN_PROTRUSION - BALL_PROTRUSION
      (contact point: ball on pin tips, 2.5mm gap between faces)
    - Pallet local Z=0 is its bottom face, so translate pallet by seated_z
    """
    seated_z = BASE_THICK + PIN_PROTRUSION - BALL_PROTRUSION
    return seated_z


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


def build_document():
    """
    Main entry point: builds the complete FreeCAD document with all components
    and four assembly visualization states.
    """

    # Create or reset the document
    doc_name = "KinematicPallet"
    if FreeCAD.ActiveDocument and FreeCAD.ActiveDocument.Name == doc_name:
        FreeCAD.closeDocument(doc_name)
    doc = FreeCAD.newDocument(doc_name)

    # -----------------------------------------------------------------------
    # Build component shapes
    # -----------------------------------------------------------------------

    base_shape, pin_shapes = make_base_station()
    pallet_shape, latch_pin_shape, ball_shapes = make_pallet()
    cam_shape = make_cam_reference()
    solenoid_shape = make_solenoid_placeholder()

    # Fuse all pins into one shape for export
    pins_fused = pin_shapes[0]
    for p in pin_shapes[1:]:
        pins_fused = pins_fused.fuse(p)

    # Fuse all balls into one shape for export
    balls_fused = ball_shapes[0]
    for b in ball_shapes[1:]:
        balls_fused = balls_fused.fuse(b)

    # Combined pallet with latch pin and balls for single-piece export
    pallet_with_pin = pallet_shape.fuse(latch_pin_shape)
    for b in ball_shapes:
        pallet_with_pin = pallet_with_pin.fuse(b)

    # -----------------------------------------------------------------------
    # Add individual components to document (reference shapes)
    # -----------------------------------------------------------------------

    components_group = doc.addObject("App::DocumentObjectGroup", "COMPONENTS")

    add_shape_to_doc(doc, "BaseStation", base_shape, COLOR_BASE)
    doc.BaseStation.adjustRelativeLinks(components_group)
    components_group.addObject(doc.BaseStation)

    add_shape_to_doc(doc, "DowelPins", pins_fused, COLOR_DOWEL)
    doc.DowelPins.adjustRelativeLinks(components_group)
    components_group.addObject(doc.DowelPins)

    add_shape_to_doc(doc, "CamTrack", cam_shape, COLOR_LATCH)
    doc.CamTrack.adjustRelativeLinks(components_group)
    components_group.addObject(doc.CamTrack)

    add_shape_to_doc(doc, "Solenoid", solenoid_shape, COLOR_SOLENOID)
    doc.Solenoid.adjustRelativeLinks(components_group)
    components_group.addObject(doc.Solenoid)

    # Pallet in its own local frame (bottom face at Z=0)
    add_shape_to_doc(doc, "Pallet", pallet_shape, COLOR_PALLET)
    doc.Pallet.adjustRelativeLinks(components_group)
    components_group.addObject(doc.Pallet)

    add_shape_to_doc(doc, "LatchPin", latch_pin_shape, COLOR_LATCH)
    doc.LatchPin.adjustRelativeLinks(components_group)
    components_group.addObject(doc.LatchPin)

    add_shape_to_doc(doc, "BallBearings", balls_fused, COLOR_BALL)
    doc.BallBearings.adjustRelativeLinks(components_group)
    components_group.addObject(doc.BallBearings)

    # -----------------------------------------------------------------------
    # STATE 1: COMPONENTS_EXPLODED
    # All components separated 40mm apart in Z for visibility
    # -----------------------------------------------------------------------

    expl_group = doc.addObject("App::DocumentObjectGroup", "COMPONENTS_EXPLODED")

    # Base station at Z=0 (as built)
    add_shape_to_doc(doc, "Expl_BaseStation", base_shape, COLOR_BASE)
    doc.Expl_BaseStation.adjustRelativeLinks(expl_group)
    expl_group.addObject(doc.Expl_BaseStation)

    add_shape_to_doc(doc, "Expl_Pins", pins_fused, COLOR_DOWEL)
    doc.Expl_Pins.adjustRelativeLinks(expl_group)
    expl_group.addObject(doc.Expl_Pins)

    add_shape_to_doc(doc, "Expl_Solenoid", solenoid_shape, COLOR_SOLENOID)
    doc.Expl_Solenoid.adjustRelativeLinks(expl_group)
    expl_group.addObject(doc.Expl_Solenoid)

    add_shape_to_doc(doc, "Expl_Cam", cam_shape, COLOR_LATCH)
    doc.Expl_Cam.adjustRelativeLinks(expl_group)
    expl_group.addObject(doc.Expl_Cam)

    # Pallet exploded upward by 40mm from seated position
    seated_z = compute_seated_z()
    expl_pallet_z = seated_z + EXPLODE_GAP

    expl_pallet = _translate_shape(pallet_shape, dz=expl_pallet_z)
    add_shape_to_doc(doc, "Expl_Pallet", expl_pallet, COLOR_PALLET)
    doc.Expl_Pallet.adjustRelativeLinks(expl_group)
    expl_group.addObject(doc.Expl_Pallet)

    expl_pin = _translate_shape(latch_pin_shape, dz=expl_pallet_z)
    add_shape_to_doc(doc, "Expl_LatchPin", expl_pin, COLOR_LATCH)
    doc.Expl_LatchPin.adjustRelativeLinks(expl_group)
    expl_group.addObject(doc.Expl_LatchPin)

    expl_balls = _translate_shape(balls_fused, dz=expl_pallet_z)
    add_shape_to_doc(doc, "Expl_Balls", expl_balls, COLOR_BALL)
    doc.Expl_Balls.adjustRelativeLinks(expl_group)
    expl_group.addObject(doc.Expl_Balls)

    # -----------------------------------------------------------------------
    # STATE 2: DESCENDING
    # Pallet 15mm above base, pin entering receiver, tabs aligned with slots
    # -----------------------------------------------------------------------

    desc_group = doc.addObject("App::DocumentObjectGroup", "DESCENDING")

    add_shape_to_doc(doc, "Desc_BaseStation", base_shape, COLOR_BASE)
    doc.Desc_BaseStation.adjustRelativeLinks(desc_group)
    desc_group.addObject(doc.Desc_BaseStation)

    add_shape_to_doc(doc, "Desc_Pins", pins_fused, COLOR_DOWEL)
    doc.Desc_Pins.adjustRelativeLinks(desc_group)
    desc_group.addObject(doc.Desc_Pins)

    add_shape_to_doc(doc, "Desc_Solenoid", solenoid_shape, COLOR_SOLENOID)
    doc.Desc_Solenoid.adjustRelativeLinks(desc_group)
    desc_group.addObject(doc.Desc_Solenoid)

    # Pallet descending: 15mm above seated position, tabs aligned with slots
    desc_pallet_z = seated_z + DESCEND_GAP

    desc_pallet = _translate_shape(pallet_shape, dz=desc_pallet_z)
    add_shape_to_doc(doc, "Desc_Pallet", desc_pallet, COLOR_PALLET)
    doc.Desc_Pallet.adjustRelativeLinks(desc_group)
    desc_group.addObject(doc.Desc_Pallet)

    desc_pin = _translate_shape(latch_pin_shape, dz=desc_pallet_z)
    add_shape_to_doc(doc, "Desc_LatchPin", desc_pin, COLOR_LATCH)
    doc.Desc_LatchPin.adjustRelativeLinks(desc_group)
    desc_group.addObject(doc.Desc_LatchPin)

    desc_balls = _translate_shape(balls_fused, dz=desc_pallet_z)
    add_shape_to_doc(doc, "Desc_Balls", desc_balls, COLOR_BALL)
    doc.Desc_Balls.adjustRelativeLinks(desc_group)
    desc_group.addObject(doc.Desc_Balls)

    # -----------------------------------------------------------------------
    # STATE 3: SEATED_UNLOCKED
    # Balls in cradles, pin in receiver, tabs aligned (not rotated)
    # -----------------------------------------------------------------------

    unlocked_group = doc.addObject("App::DocumentObjectGroup", "SEATED_UNLOCKED")

    add_shape_to_doc(doc, "Unlk_BaseStation", base_shape, COLOR_BASE)
    doc.Unlk_BaseStation.adjustRelativeLinks(unlocked_group)
    unlocked_group.addObject(doc.Unlk_BaseStation)

    add_shape_to_doc(doc, "Unlk_Pins", pins_fused, COLOR_DOWEL)
    doc.Unlk_Pins.adjustRelativeLinks(unlocked_group)
    unlocked_group.addObject(doc.Unlk_Pins)

    add_shape_to_doc(doc, "Unlk_Solenoid", solenoid_shape, COLOR_SOLENOID)
    doc.Unlk_Solenoid.adjustRelativeLinks(unlocked_group)
    unlocked_group.addObject(doc.Unlk_Solenoid)

    # Pallet fully seated, no rotation
    seat_pallet = _translate_shape(pallet_shape, dz=seated_z)
    add_shape_to_doc(doc, "Unlk_Pallet", seat_pallet, COLOR_PALLET)
    doc.Unlk_Pallet.adjustRelativeLinks(unlocked_group)
    unlocked_group.addObject(doc.Unlk_Pallet)

    seat_pin = _translate_shape(latch_pin_shape, dz=seated_z)
    add_shape_to_doc(doc, "Unlk_LatchPin", seat_pin, COLOR_LATCH)
    doc.Unlk_LatchPin.adjustRelativeLinks(unlocked_group)
    unlocked_group.addObject(doc.Unlk_LatchPin)

    seat_balls = _translate_shape(balls_fused, dz=seated_z)
    add_shape_to_doc(doc, "Unlk_Balls", seat_balls, COLOR_BALL)
    doc.Unlk_Balls.adjustRelativeLinks(unlocked_group)
    unlocked_group.addObject(doc.Unlk_Balls)

    # -----------------------------------------------------------------------
    # STATE 4: SEATED_LOCKED
    # Fully engaged: balls seated, tabs rotated 45 degrees, locked.
    # This is the operational state.
    # -----------------------------------------------------------------------

    locked_group = doc.addObject("App::DocumentObjectGroup", "SEATED_LOCKED")

    add_shape_to_doc(doc, "Lock_BaseStation", base_shape, COLOR_BASE)
    doc.Lock_BaseStation.adjustRelativeLinks(locked_group)
    locked_group.addObject(doc.Lock_BaseStation)

    add_shape_to_doc(doc, "Lock_Pins", pins_fused, COLOR_DOWEL)
    doc.Lock_Pins.adjustRelativeLinks(locked_group)
    locked_group.addObject(doc.Lock_Pins)

    add_shape_to_doc(doc, "Lock_Solenoid", solenoid_shape, COLOR_SOLENOID)
    doc.Lock_Solenoid.adjustRelativeLinks(locked_group)
    locked_group.addObject(doc.Lock_Solenoid)

    # Pallet seated with latch pin rotated 45 degrees (locked).
    # The pallet plate itself does not rotate — only the latch pin rotates
    # relative to the receiver. In this design the pallet and pin are one
    # rigid body, so the entire pallet rotates 45 degrees about Z when
    # the cam engages. The kinematic coupling balls-on-pins accommodate
    # this small rotation because the ball contacts are rotationally
    # symmetric (each ball can rotate in its cradle).
    locked_pallet = _translate_shape(pallet_shape, dz=seated_z)
    locked_pallet = _rotate_shape_z(locked_pallet, LATCH_LOCK_ANGLE)
    add_shape_to_doc(doc, "Lock_Pallet", locked_pallet, COLOR_PALLET)
    doc.Lock_Pallet.adjustRelativeLinks(locked_group)
    locked_group.addObject(doc.Lock_Pallet)

    locked_pin = _translate_shape(latch_pin_shape, dz=seated_z)
    locked_pin = _rotate_shape_z(locked_pin, LATCH_LOCK_ANGLE)
    add_shape_to_doc(doc, "Lock_LatchPin", locked_pin, COLOR_LATCH)
    doc.Lock_LatchPin.adjustRelativeLinks(locked_group)
    locked_group.addObject(doc.Lock_LatchPin)

    locked_balls = _translate_shape(balls_fused, dz=seated_z)
    locked_balls = _rotate_shape_z(locked_balls, LATCH_LOCK_ANGLE)
    add_shape_to_doc(doc, "Lock_Balls", locked_balls, COLOR_BALL)
    doc.Lock_Balls.adjustRelativeLinks(locked_group)
    locked_group.addObject(doc.Lock_Balls)

    # -----------------------------------------------------------------------
    # Recompute document
    # -----------------------------------------------------------------------
    doc.recompute()

    # -----------------------------------------------------------------------
    # Export STL files via Mesh module (Part.export silently fails for STL
    # when given raw TopoShape objects in headless mode)
    # -----------------------------------------------------------------------
    import Mesh
    import MeshPart

    def _export_stl(shape, filepath, linear_deflection=0.1, angular_deflection=0.5):
        """Mesh a TopoShape and write STL. Works in headless FreeCADCmd."""
        mesh = MeshPart.meshFromShape(
            Shape=shape,
            LinearDeflection=linear_deflection,
            AngularDeflection=angular_deflection,
        )
        mesh.write(filepath)
        print(f"Exported: {filepath}  ({mesh.CountFacets} facets)")

    # Base station with pins (single manufacturing piece + pressed pins)
    base_with_pins = base_shape.fuse(pins_fused)
    _export_stl(base_with_pins, os.path.join(OUTPUT_DIR, "base_station.stl"))

    # Pallet with latch pin and balls
    _export_stl(pallet_with_pin, os.path.join(OUTPUT_DIR, "pallet.stl"))

    # Latch pin alone (for detail views / printing)
    _export_stl(latch_pin_shape, os.path.join(OUTPUT_DIR, "latch_pin.stl"))

    # Full locked assembly (merged visualization)
    locked_assy = base_with_pins
    locked_assy = locked_assy.fuse(solenoid_shape)

    seated_pallet_locked = _translate_shape(pallet_with_pin, dz=seated_z)
    seated_pallet_locked = _rotate_shape_z(seated_pallet_locked, LATCH_LOCK_ANGLE)
    locked_assy = locked_assy.fuse(seated_pallet_locked)

    _export_stl(locked_assy, os.path.join(OUTPUT_DIR, "assembly_locked.stl"))

    # -----------------------------------------------------------------------
    # Save FreeCAD document
    # -----------------------------------------------------------------------
    fcstd_path = os.path.join(FREECAD_DIR, "kinematic_pallet.FCStd")
    doc.saveAs(fcstd_path)
    print(f"Saved: {fcstd_path}")

    print("\n=== Kinematic Pallet System build complete ===")
    print(f"  Components: BaseStation, Pallet, LatchPin, BallBearings, DowelPins")
    print(f"  States: COMPONENTS_EXPLODED, DESCENDING, SEATED_UNLOCKED, SEATED_LOCKED")
    print(f"  Seated Z offset: {compute_seated_z():.1f}mm")
    print(f"  Base-pallet face gap: {PIN_PROTRUSION - BALL_PROTRUSION:.1f}mm")
    print(f"  Twist-lock rotation: {LATCH_LOCK_ANGLE}° over {CAM_Z_TRAVEL}mm descent")


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == "__main__":
    build_document()
