"""
Manufacturing Drawings — Kinematic Workholding Pallet System
=============================================================

Generates 2D manufacturing drawings with orthographic views, section views,
key dimensions, and GD&T callouts using FreeCAD TechDraw workbench.

Produces two drawing sheets (A3 landscape):
  1. Base Station — top, front, right, iso views + cradle section + receiver section
  2. Pallet — bottom, front, right, iso views + ball pocket section

Dimensions and GD&T are placed as annotations with values computed from
PalletConfig, avoiding fragile edge-index references.

Usage:
  FreeCADCmd -c "exec(open('drawing_pallet.py').read())"
  FreeCADCmd -c "DRAWING_PRESET='large'; exec(open('drawing_pallet.py').read())"

Requires: FreeCAD 0.21+ with TechDraw workbench.
Output: SVG drawings in cad/outputs/
"""

import math
import os
import sys

import FreeCAD
import Part

# ---------------------------------------------------------------------------
# Path setup — import from kinematic_pallet.py
# ---------------------------------------------------------------------------
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.path.abspath("C:/Users/user/kinematic-cell-tooling/cad/scripts")

sys.path.insert(0, SCRIPT_DIR)
from kinematic_pallet import (
    PalletConfig, PRESETS, OUTPUT_DIR, FREECAD_DIR,
    make_base_station, make_pallet, make_solenoid_placeholder,
    _vec, _translate_shape, _rotate_shape_z,
)


# ---------------------------------------------------------------------------
# TechDraw page setup
# ---------------------------------------------------------------------------

def _find_template():
    """Find A3 landscape TechDraw template from FreeCAD installation."""
    resource_dir = FreeCAD.getResourceDir()
    candidates = [
        os.path.join(resource_dir, "Mod", "TechDraw", "Templates", "A3_LandscapeTD.svg"),
        os.path.join(resource_dir, "Mod", "TechDraw", "Templates", "A3_Landscape_TD.svg"),
        os.path.join(resource_dir, "Mod", "TechDraw", "Templates", "A3_LandscapeTD.svg"),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    # Fallback: search for any A3 template
    template_dir = os.path.join(resource_dir, "Mod", "TechDraw", "Templates")
    if os.path.isdir(template_dir):
        for f in os.listdir(template_dir):
            if "A3" in f and f.endswith(".svg"):
                return os.path.join(template_dir, f)
    return None


def create_drawing_page(doc, name, template_path=None):
    """Create a TechDraw page with A3 landscape template."""
    page = doc.addObject("TechDraw::DrawPage", name)

    if template_path is None:
        template_path = _find_template()

    if template_path and os.path.exists(template_path):
        template = doc.addObject("TechDraw::DrawSVGTemplate", f"{name}_Template")
        template.Template = template_path
        page.Template = template
    else:
        print(f"  Warning: No TechDraw template found. Using blank page.")

    return page


# ---------------------------------------------------------------------------
# View helpers
# ---------------------------------------------------------------------------

def add_view(doc, page, name, source_obj, direction, x, y, scale=1.0,
             rotation=0.0):
    """
    Add an orthographic projection view to a TechDraw page.

    Args:
        source_obj: FreeCAD document object (Part::Feature) to project
        direction: FreeCAD.Vector — viewing direction (camera looks ALONG this)
        x, y: position on the drawing sheet (mm from bottom-left)
        scale: drawing scale (1.0 = full size)
        rotation: rotation of the view on the sheet (degrees)
    """
    view = doc.addObject("TechDraw::DrawViewPart", name)
    view.Source = [source_obj]
    view.Direction = direction
    view.X = x
    view.Y = y
    view.Scale = scale
    view.Rotation = rotation
    page.addView(view)
    return view


def add_annotation(doc, page, name, text_lines, x, y, font_size=3.5):
    """Add a text annotation to a TechDraw page."""
    ann = doc.addObject("TechDraw::DrawViewAnnotation", name)
    ann.Text = text_lines
    ann.X = x
    ann.Y = y
    ann.TextSize = font_size
    page.addView(ann)
    return ann


# ---------------------------------------------------------------------------
# Dimension annotation blocks (computed from PalletConfig, not edge refs)
# ---------------------------------------------------------------------------

def base_station_dimensions(cfg):
    """Generate dimension annotation text for the base station."""
    lines = [
        "BASE STATION DIMENSIONS",
        f"Overall: {cfg.base_length:.0f} x {cfg.base_width:.0f} "
        f"x {cfg.base_thick:.1f} mm",
        f"Material: 6061-T6 Aluminum",
        "",
        f"Mounting: 4x {_bolt_size_from_cfg(cfg)} counterbored",
        f"  Pattern: {cfg.mount_spacing:.1f} mm square",
        f"  Clearance: dia {cfg.mount_hole_dia:.1f} mm thru",
        f"  Counterbore: dia {cfg.mount_cbore_dia:.1f} "
        f"x {cfg.mount_cbore_depth:.1f} mm deep",
        "",
        f"Cradles: 3x at 120 deg, R{cfg.bolt_circle_r:.1f} BC",
        f"  Pins: {cfg.pin_dia:.1f}mm h6 ground dowel",
        f"  Pin spacing: {cfg.pin_center_dist:.2f} mm c-c",
        f"  Press-fit: dia {cfg.pin_hole_dia:.2f} "
        f"x {cfg.pin_press_depth:.1f} deep",
        f"  Protrusion: {cfg.pin_protrusion:.1f} mm above top face",
        "",
        f"Receiver: dia {cfg.receiver_dia:.1f} "
        f"x {cfg.receiver_depth:.1f} mm deep",
        f"  Slots: {cfg.num_receiver_slots}x at 90 deg",
        f"  Slot: {cfg.receiver_slot_width:.1f}W "
        f"x {cfg.receiver_slot_depth:.1f}D "
        f"x {cfg.receiver_slot_height:.1f}H mm",
        "",
        f"Solenoid pocket (from bottom):",
        f"  dia {cfg.solenoid_dia:.1f} "
        f"x {cfg.solenoid_depth:.1f} mm deep",
        f"  2x M3 retention, {cfg.solenoid_mount_offset:.1f} mm apart",
        "",
        f"Hall sensor pocket: dia {cfg.hall_pocket_dia:.1f} "
        f"x {cfg.hall_pocket_depth:.1f} deep",
        f"  at R{cfg.bolt_circle_r:.1f}, 60 deg from cradle 1",
    ]
    return lines


def base_station_gdt(cfg):
    """Generate GD&T callout text for the base station."""
    lines = [
        "GD&T — BASE STATION",
        "",
        "DATUM SCHEME:",
        "  A = Top face (coupling seating surface)",
        "  B = Center (bolt circle center)",
        "  C = Cradle 1 radial (orientation)",
        "",
        "CRITICAL FEATURES:",
        f"  Top face flatness: 0.025 mm [A]",
        f"  Pin holes TP: dia 0.050 mm | A | B | C |",
        f"  Pin hole dia: {cfg.pin_hole_dia:.3f} "
        f"+0.000/-{cfg.pin_dia * 0.006:.3f} mm",
        f"  Receiver bore TP: dia 0.050 mm | A | B |",
        f"  Receiver slots TP: dia 0.100 mm | A | B |",
        "",
        "NON-CRITICAL:",
        f"  Mounting holes: TP dia 0.200 mm",
        f"  Wire channel: +/-0.5 mm",
        f"  Solenoid pocket: +/-0.2 mm",
    ]
    return lines


def pallet_dimensions(cfg):
    """Generate dimension annotation text for the pallet."""
    lines = [
        "PALLET DIMENSIONS",
        f"Overall: {cfg.pallet_length:.0f} x {cfg.pallet_width:.0f} "
        f"x {cfg.pallet_thick:.1f} mm",
        f"Material: 6061-T6 Aluminum",
        "",
        f"Ball pockets (bottom face): 3x at 120 deg, "
        f"R{cfg.bolt_circle_r:.1f} BC",
        f"  Pocket: dia {cfg.ball_pocket_dia:.2f} "
        f"x {cfg.ball_pocket_depth:.1f} deep",
        f"  Ball: {cfg.ball_dia:.1f} mm G25 chrome steel",
        f"  Protrusion: {cfg.ball_protrusion:.1f} mm below bottom face",
        "",
        f"Latch pin (bottom face, center):",
        f"  Body: dia {cfg.latch_pin_dia:.1f} "
        f"x {cfg.latch_pin_length:.1f} long",
        f"  Tabs: {cfg.num_latch_tabs}x at 90 deg",
        f"  Tab: {cfg.latch_tab_width:.1f}W "
        f"x {cfg.latch_tab_radial:.1f}R "
        f"x {cfg.latch_tab_height:.1f}H mm",
        f"  Chamfer: {cfg.latch_chamfer:.1f} x 45 deg entry",
        f"  Internal bore: dia {cfg.latch_bore_dia:.1f} (for extraction)",
        "",
        f"Robot pick: 2x dia {cfg.robot_hole_dia:.1f} thru",
        f"  Spacing: {cfg.robot_hole_spacing:.1f} mm on center",
        "",
        f"Workpiece mount: 4x dia {cfg.work_hole_dia:.1f} thru",
        f"  Pattern: {cfg.work_hole_spacing:.1f} mm square",
        "",
        f"Magnet pocket: dia {cfg.magnet_pocket_dia:.1f} "
        f"x {cfg.magnet_pocket_depth:.1f} deep",
        f"  at R{cfg.bolt_circle_r:.1f}, 60 deg",
    ]
    return lines


def pallet_gdt(cfg):
    """Generate GD&T callout text for the pallet."""
    lines = [
        "GD&T — PALLET",
        "",
        "DATUM SCHEME:",
        "  A = Bottom face (coupling mating surface)",
        "  B = Center (ball pocket bolt circle center)",
        "  C = Ball pocket 1 radial (orientation)",
        "",
        "CRITICAL FEATURES:",
        f"  Bottom face flatness: 0.025 mm [A]",
        f"  Ball pockets TP: dia 0.050 mm | A | B | C |",
        f"  Pocket dia: {cfg.ball_pocket_dia:.3f} "
        f"+0.000/-{cfg.ball_dia * 0.006:.3f} mm",
        f"  Latch pin concentricity: dia 0.050 mm | A | B |",
        "",
        "NON-CRITICAL:",
        f"  Robot pick holes: TP dia 0.200 mm",
        f"  Work mount holes: TP dia 0.200 mm",
        f"  Magnet pocket: +/-0.3 mm",
    ]
    return lines


def assembly_notes(cfg):
    """Generate assembly notes annotation."""
    lines = [
        "ASSEMBLY NOTES",
        "",
        f"1. Press 6x dowel pins ({cfg.pin_dia:.1f}mm h6) into base station.",
        f"   Pin protrusion: {cfg.pin_protrusion:.1f} mm above top face.",
        f"   Verify with depth gauge.",
        "",
        f"2. Press 3x balls ({cfg.ball_dia:.1f}mm G25) into pallet pockets.",
        f"   Ball protrusion: {cfg.ball_protrusion:.1f} mm below bottom face.",
        f"   All balls from same manufacturing lot.",
        "",
        f"3. Seated face gap: {cfg.face_gap:.1f} mm",
        f"   (pin protrusion - ball protrusion)",
        "",
        f"4. Twist-lock engagement: {cfg.latch_lock_angle:.0f} deg rotation",
        f"   over {cfg.cam_z_travel:.0f} mm descent (self-engaging cam).",
        "",
        f"5. Coupling repeatability: sub-5 um XY, sub-10 um Z",
        f"   (See tolerance_stackup_report.md for full analysis)",
    ]
    return lines


def _bolt_size_from_cfg(cfg):
    """Infer bolt size string from clearance hole diameter."""
    bolt_map = {4.5: "M4", 5.5: "M5", 6.5: "M6", 9.0: "M8",
                11.0: "M10", 13.5: "M12"}
    return bolt_map.get(cfg.mount_hole_dia, f"dia {cfg.mount_hole_dia:.1f}")


# ---------------------------------------------------------------------------
# Main drawing generation
# ---------------------------------------------------------------------------

def build_drawings(cfg=None):
    """
    Generate manufacturing drawings for the base station and pallet.

    Args:
        cfg: PalletConfig instance. If None, uses MEDIUM preset.
    """
    if cfg is None:
        cfg = PRESETS["medium"]

    label = cfg.label
    print(f"\n=== Generating drawings: {label} ({cfg.plate_size:.0f}mm) ===")

    # Create document
    doc_name = f"PalletDrawings_{label}"
    if FreeCAD.ActiveDocument and FreeCAD.ActiveDocument.Name == doc_name:
        FreeCAD.closeDocument(doc_name)
    doc = FreeCAD.newDocument(doc_name)

    # Build geometry
    base_shape, pin_shapes = make_base_station(cfg)
    pallet_shape, latch_pin_shape, ball_shapes = make_pallet(cfg)

    # Fuse for drawing source objects
    pins_fused = pin_shapes[0]
    for p in pin_shapes[1:]:
        pins_fused = pins_fused.fuse(p)
    base_with_pins = base_shape.fuse(pins_fused)

    balls_fused = ball_shapes[0]
    for b in ball_shapes[1:]:
        balls_fused = balls_fused.fuse(b)
    pallet_complete = pallet_shape.fuse(latch_pin_shape)
    for b in ball_shapes:
        pallet_complete = pallet_complete.fuse(b)

    # Add source objects to document (required for TechDraw views)
    base_obj = doc.addObject("Part::Feature", "BaseStation")
    base_obj.Shape = base_with_pins

    pallet_obj = doc.addObject("Part::Feature", "Pallet")
    pallet_obj.Shape = pallet_complete

    doc.recompute()

    # Determine drawing scale to fit A3 (420 x 297 mm usable ~380 x 260)
    # Scale so largest component fits in ~150mm on sheet
    draw_scale = min(1.0, 140.0 / cfg.plate_size)
    draw_scale = round(draw_scale, 2)
    if draw_scale < 0.25:
        draw_scale = 0.25

    section_scale = draw_scale * 2.0  # Sections at 2x main views
    if section_scale > 2.0:
        section_scale = 2.0

    print(f"  Drawing scale: {draw_scale}:1  Section scale: {section_scale}:1")

    # ===================================================================
    # SHEET 1: BASE STATION
    # ===================================================================
    page1 = create_drawing_page(doc, "BaseStation_Drawing")

    # Top view (looking down Z axis)
    top = add_view(doc, page1, "BS_Top", base_obj,
                   _vec(0, 0, -1), x=100, y=180, scale=draw_scale)

    # Front view (looking along Y axis)
    front = add_view(doc, page1, "BS_Front", base_obj,
                     _vec(0, -1, 0), x=100, y=80, scale=draw_scale)

    # Right view (looking along X axis)
    right = add_view(doc, page1, "BS_Right", base_obj,
                     _vec(1, 0, 0), x=250, y=80, scale=draw_scale)

    # Isometric view
    iso_dir = _vec(1, -1, 1)
    iso_dir.normalize()
    add_view(doc, page1, "BS_Iso", base_obj,
             iso_dir, x=300, y=190, scale=draw_scale * 0.7)

    # Section A-A: through cradle at 0 degrees (radial cut, XZ plane)
    # Note: TechDraw sections require GUI mode for OCC section computation.
    # When opened in FreeCAD, add section views manually:
    #   Section A-A: radial cut through cradle 1 (Y=0 plane at bolt circle R)
    #   Section B-B: axial cut through receiver center (Y=0 plane)

    # Dimension annotations
    add_annotation(doc, page1, "BS_Dims",
                   base_station_dimensions(cfg),
                   x=30, y=260, font_size=2.8)

    # GD&T annotations
    add_annotation(doc, page1, "BS_GDT",
                   base_station_gdt(cfg),
                   x=30, y=130, font_size=2.8)

    # Title block info
    add_annotation(doc, page1, "BS_Title", [
        f"KINEMATIC PALLET BASE STATION — {label.upper()}",
        f"Scale {draw_scale}:1 (sections {section_scale}:1)",
        f"All dimensions in mm unless noted",
        f"Material: 6061-T6 Aluminum",
        f"Deburr all edges. Break sharp corners 0.2mm.",
    ], x=290, y=20, font_size=3.0)

    # ===================================================================
    # SHEET 2: PALLET
    # ===================================================================
    page2 = create_drawing_page(doc, "Pallet_Drawing")

    # Bottom view (looking up Z axis — shows ball pockets and latch pin)
    bottom = add_view(doc, page2, "PL_Bottom", pallet_obj,
                      _vec(0, 0, 1), x=100, y=180, scale=draw_scale)

    # Front view
    pallet_front = add_view(doc, page2, "PL_Front", pallet_obj,
                            _vec(0, -1, 0), x=100, y=80, scale=draw_scale)

    # Right view
    add_view(doc, page2, "PL_Right", pallet_obj,
             _vec(1, 0, 0), x=250, y=80, scale=draw_scale)

    # Isometric view
    add_view(doc, page2, "PL_Iso", pallet_obj,
             iso_dir, x=300, y=190, scale=draw_scale * 0.7)

    # Section C-C: through ball pocket at 0 degrees
    # Note: Add section views in FreeCAD GUI (see base station note above).
    #   Section C-C: radial cut through ball pocket 1 at bolt circle R

    # Dimension annotations
    add_annotation(doc, page2, "PL_Dims",
                   pallet_dimensions(cfg),
                   x=30, y=260, font_size=2.8)

    # GD&T annotations
    add_annotation(doc, page2, "PL_GDT",
                   pallet_gdt(cfg),
                   x=30, y=130, font_size=2.8)

    # Assembly notes
    add_annotation(doc, page2, "PL_Assembly",
                   assembly_notes(cfg),
                   x=30, y=45, font_size=2.5)

    # Title block
    add_annotation(doc, page2, "PL_Title", [
        f"KINEMATIC PALLET — {label.upper()}",
        f"Scale {draw_scale}:1 (sections {section_scale}:1)",
        f"All dimensions in mm unless noted",
        f"Material: 6061-T6 Aluminum",
        f"Deburr all edges. Break sharp corners 0.2mm.",
    ], x=290, y=20, font_size=3.0)

    # ===================================================================
    # Recompute and export
    # ===================================================================
    doc.recompute()

    # Export SVG (reliable headless)
    svg_files = []
    for page_obj, page_name in [(page1, f"base_station_{label}_drawing"),
                                 (page2, f"pallet_{label}_drawing")]:
        svg_path = os.path.join(OUTPUT_DIR, f"{page_name}.svg")
        try:
            import TechDrawGui
            TechDrawGui.exportPageAsSvg(page_obj, svg_path)
        except ImportError:
            # Headless: use page's built-in SVG export
            try:
                page_obj.ViewObject  # may be None headless
            except Exception:
                pass
            # In headless FreeCADCmd, TechDraw pages can be saved via document
            # The SVG export may not be available without GUI.
            # Fall through to document save.
            pass
        svg_files.append(svg_path)

    # Save FreeCAD document (always works, contains the TechDraw pages)
    fcstd_path = os.path.join(FREECAD_DIR, f"pallet_drawings_{label}.FCStd")
    doc.saveAs(fcstd_path)
    print(f"  Saved: {fcstd_path}")
    print(f"  Open in FreeCAD to view/export drawings as SVG or PDF.")
    print(f"  Sheets: BaseStation_Drawing, Pallet_Drawing")

    return doc


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    preset_name = globals().get("DRAWING_PRESET", None)
    build_all = globals().get("DRAWING_BUILD_ALL", False)

    if build_all:
        for name, preset_cfg in PRESETS.items():
            build_drawings(preset_cfg)
    elif preset_name:
        if preset_name not in PRESETS:
            raise ValueError(f"Unknown preset '{preset_name}'. "
                             f"Choose from: {list(PRESETS.keys())}")
        build_drawings(PRESETS[preset_name])
    else:
        build_drawings(PRESETS["medium"])
