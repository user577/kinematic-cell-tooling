#!/usr/bin/env python3
"""
Tolerance Stack-Up Analysis — Kinematic Workholding Pallet System
=================================================================

Standalone script (no FreeCAD dependency). Predicts coupling repeatability
and accuracy from component tolerances using Hertzian contact theory,
RSS statistical stack-up, and worst-case analysis.

Key insight: REPEATABILITY and ACCURACY are different things.
  - Repeatability: how well the coupling returns to the same position
    each time it is engaged. Dominated by contact centroid scatter
    (ball sphericity, surface finish, contamination).
  - Accuracy: deviation from nominal position. Dominated by machining
    position tolerances (hole positions, pocket positions).

The coupling hardware (G25 balls, h6 pins) gives you sub-micrometer
repeatability essentially for free. You buy accuracy with machining.

Usage:
  python tolerance_stackup.py                   # medium preset
  python tolerance_stackup.py --preset large    # named preset
  python tolerance_stackup.py --all             # all presets comparison
  python tolerance_stackup.py --output report.md
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

# ---------------------------------------------------------------------------
# Standard hardware tables (duplicated from kinematic_pallet.py for
# FreeCAD-free operation)
# ---------------------------------------------------------------------------

# ISO 3290 ball bearing grade specifications
BALL_GRADES = {
    "G3":   {"sphericity_um": 0.08, "dia_tol_um": 0.08, "Ra_um": 0.005},
    "G5":   {"sphericity_um": 0.13, "dia_tol_um": 0.13, "Ra_um": 0.008},
    "G10":  {"sphericity_um": 0.25, "dia_tol_um": 0.25, "Ra_um": 0.012},
    "G16":  {"sphericity_um": 0.40, "dia_tol_um": 0.40, "Ra_um": 0.016},
    "G25":  {"sphericity_um": 0.625, "dia_tol_um": 0.625, "Ra_um": 0.020},
    "G40":  {"sphericity_um": 1.0,  "dia_tol_um": 1.0,  "Ra_um": 0.032},
    "G100": {"sphericity_um": 2.5,  "dia_tol_um": 2.5,  "Ra_um": 0.050},
    "G200": {"sphericity_um": 5.0,  "dia_tol_um": 5.0,  "Ra_um": 0.100},
    "G500": {"sphericity_um": 12.5, "dia_tol_um": 12.5, "Ra_um": 0.200},
}

# ISO 286 h6 shaft tolerance (upper deviation always 0, lower is negative)
# Key: nominal diameter (mm), Value: (upper_um, lower_um)
PIN_H6_TOL_UM = {
    2.0:  (0, -6),
    3.0:  (0, -6),
    4.0:  (0, -8),
    5.0:  (0, -8),
    6.0:  (0, -8),
    8.0:  (0, -9),
    10.0: (0, -9),
    12.0: (0, -11),
    15.0: (0, -11),
}

# Material properties
CTE_ALUMINUM = 23.1e-6  # m/m/degC, 6061-T6
E_STEEL_GPA = 210.0
NU_STEEL = 0.29


# ---------------------------------------------------------------------------
# Minimal geometry config (mirrors PalletConfig derivations)
# ---------------------------------------------------------------------------

@dataclass
class GeometryConfig:
    """Coupling geometry derived from master inputs, no FreeCAD needed."""
    label: str
    plate_size_mm: float
    ball_dia_mm: float
    pin_dia_mm: float
    bolt_circle_r_mm: float
    pin_center_dist_mm: float
    base_thick_mm: float
    pallet_thick_mm: float

    @classmethod
    def from_master(cls, plate_size, ball_dia, label="custom"):
        """Derive geometry from plate size and ball diameter."""
        # Pin diameter: match ball (same logic as PalletConfig)
        std_pins = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0]
        pin_dia = max(p for p in std_pins if p <= ball_dia)
        bolt_circle_r = round(plate_size * 0.292, 1)
        pin_center_dist = (ball_dia / 2 + pin_dia / 2) * math.sqrt(2)
        base_thick = max(12.0, round(plate_size * 0.167, 1))
        pallet_thick = max(6.0, round(plate_size * 0.083, 1))
        return cls(
            label=label,
            plate_size_mm=plate_size,
            ball_dia_mm=ball_dia,
            pin_dia_mm=pin_dia,
            bolt_circle_r_mm=bolt_circle_r,
            pin_center_dist_mm=pin_center_dist,
            base_thick_mm=base_thick,
            pallet_thick_mm=pallet_thick,
        )


GEOMETRY_PRESETS = {
    "small":  GeometryConfig.from_master(80,  3.0,  "small"),
    "medium": GeometryConfig.from_master(120, 5.0,  "medium"),
    "large":  GeometryConfig.from_master(200, 8.0,  "large"),
    "xl":     GeometryConfig.from_master(300, 10.0, "xl"),
}


# ---------------------------------------------------------------------------
# Tolerance inputs
# ---------------------------------------------------------------------------

@dataclass
class ToleranceInputs:
    """All tolerance sources for the coupling."""
    ball_grade: str = "G25"
    pin_tolerance_class: str = "h6"
    machining_position_tol_mm: float = 0.025  # manual mill, TP zone radius
    machining_bore_tol_mm: float = 0.010      # reamed bore diameter tolerance
    surface_finish_pin_Ra_um: float = 0.4     # ground pin
    contamination_allowance_um: float = 0.5   # clean shop environment
    preload_per_ball_N: float = 10.0          # spring/gravity preload
    preload_variation_pct: float = 10.0       # preload force variation ±%
    delta_T_degC: float = 5.0                 # temperature swing ±


# ---------------------------------------------------------------------------
# Hertzian contact model
# ---------------------------------------------------------------------------

def hertzian_sphere_on_cylinder(ball_r_mm, pin_r_mm, force_N,
                                 E_GPa=E_STEEL_GPA, nu=NU_STEEL):
    """
    Hertzian contact for a sphere on a cylinder (convex-convex).

    Returns:
        contact_half_width_mm: semi-width of elliptical contact patch
        max_pressure_MPa: peak Hertzian contact pressure
        approach_um: elastic approach (deformation) of surfaces
    """
    # Effective radius (1/R_eff = 1/R1 + 1/R2 for convex-convex in contact plane)
    R1 = ball_r_mm * 1e-3  # m
    R2 = pin_r_mm * 1e-3   # m
    R_eff = (R1 * R2) / (R1 + R2)  # m

    # Effective modulus (same material: E* = E / (2*(1-nu^2)))
    E_star = (E_GPa * 1e9) / (2 * (1 - nu**2))  # Pa

    # For sphere on cylinder, the contact is elliptical. For similar radii,
    # approximate as circular (Hertz sphere-on-sphere equivalent).
    F = force_N

    # Contact radius (circular Hertz approximation)
    a = ((3 * F * R_eff) / (4 * E_star)) ** (1.0 / 3.0)  # m

    # Max pressure
    p_max = (3 * F) / (2 * math.pi * a**2)  # Pa

    # Approach (elastic deformation)
    delta = (a**2) / R_eff  # m

    return a * 1e3, p_max * 1e-6, delta * 1e6  # mm, MPa, um


# ---------------------------------------------------------------------------
# Tolerance analysis engine
# ---------------------------------------------------------------------------

@dataclass
class ContributorResult:
    """Single tolerance contributor result."""
    name: str
    source: str
    value_um: float
    xy_effect_um: float
    z_effect_um: float
    category: str  # "repeatability" or "accuracy"
    distribution: str  # "uniform", "gaussian", "systematic"


def analyze_coupling(geo: GeometryConfig, tol: ToleranceInputs) -> dict:
    """
    Full tolerance stack-up analysis for the kinematic coupling.

    Returns dict with:
        hertzian: contact analysis results
        contributors: list of ContributorResult
        repeatability_xy_um: RSS repeatability in XY
        repeatability_z_um: RSS repeatability in Z
        accuracy_xy_um: worst-case accuracy in XY
        accuracy_z_um: worst-case accuracy in Z
        thermal_sensitivity: um/degC in XY and Z
    """
    ball_r = geo.ball_dia_mm / 2.0
    pin_r = geo.pin_dia_mm / 2.0
    bcr = geo.bolt_circle_r_mm

    # --- Ball grade specs ---
    grade = BALL_GRADES[tol.ball_grade]
    sphericity = grade["sphericity_um"]
    ball_dia_tol = grade["dia_tol_um"]
    ball_Ra = grade["Ra_um"]

    # --- Pin tolerance ---
    pin_tol_entry = PIN_H6_TOL_UM.get(geo.pin_dia_mm)
    if pin_tol_entry is None:
        # Find nearest size
        nearest = min(PIN_H6_TOL_UM.keys(), key=lambda k: abs(k - geo.pin_dia_mm))
        pin_tol_entry = PIN_H6_TOL_UM[nearest]
    pin_upper_um, pin_lower_um = pin_tol_entry
    pin_tol_range_um = pin_upper_um - pin_lower_um  # total range

    # --- Hertzian contact ---
    contact_a, contact_p, contact_delta = hertzian_sphere_on_cylinder(
        ball_r, pin_r, tol.preload_per_ball_N
    )

    # --- Build contributor list ---
    contributors = []

    # 1. Ball sphericity — repeatability
    # Ball may rotate between engagements, presenting a different meridian.
    # Contact centroid shifts by up to sphericity/2 (high spot vs low spot).
    # Uniform distribution: sigma = range / (2*sqrt(3))
    sph_xy = sphericity / 2.0  # worst-case shift per ball
    sph_sigma = sphericity / (2 * math.sqrt(3))  # sigma for uniform dist
    # Over 3 balls in a 120-deg coupling, XY errors partially average.
    # Geometric factor: ~1/sqrt(3) for independent balls, ~1.15 for correlated.
    sph_xy_coupling = sph_sigma * 1.15  # geometric amplification
    sph_z_coupling = sph_sigma * 1.5    # Z is less constrained
    contributors.append(ContributorResult(
        name="Ball sphericity",
        source=f"ISO 3290 {tol.ball_grade}: {sphericity:.3f} um",
        value_um=sphericity,
        xy_effect_um=sph_xy_coupling,
        z_effect_um=sph_z_coupling,
        category="repeatability",
        distribution="uniform",
    ))

    # 2. Surface finish scatter — repeatability
    # Composite Ra of ball + pin surfaces. Contact centroid scatter ~ Ra/3 (Slocum).
    Ra_composite = math.sqrt(ball_Ra**2 + tol.surface_finish_pin_Ra_um**2)
    finish_sigma = Ra_composite / 3.0  # per contact
    # 6 contacts total, XY coupling factor ~sqrt(2)
    finish_xy = finish_sigma * math.sqrt(2)
    finish_z = finish_sigma * math.sqrt(3)
    contributors.append(ContributorResult(
        name="Surface finish scatter",
        source=f"Ball Ra {ball_Ra:.3f} um + Pin Ra {tol.surface_finish_pin_Ra_um:.1f} um",
        value_um=Ra_composite,
        xy_effect_um=finish_xy,
        z_effect_um=finish_z,
        category="repeatability",
        distribution="gaussian",
    ))

    # 3. Contamination allowance — repeatability
    # Dust / debris on contact surfaces shifts contact centroid.
    contam_sigma = tol.contamination_allowance_um / math.sqrt(3)
    contam_xy = contam_sigma * 1.15
    contam_z = contam_sigma * 1.5
    contributors.append(ContributorResult(
        name="Contamination allowance",
        source=f"{tol.contamination_allowance_um:.1f} um effective Ra (clean shop)",
        value_um=tol.contamination_allowance_um,
        xy_effect_um=contam_xy,
        z_effect_um=contam_z,
        category="repeatability",
        distribution="uniform",
    ))

    # 4. Preload variation — repeatability
    # Elastic deformation varies with F^(2/3). If preload varies ±pct,
    # approach varies by ±(2/3)*pct of contact_delta.
    preload_var = (2.0 / 3.0) * (tol.preload_variation_pct / 100.0) * contact_delta
    # Z effect only (approach is along nesting direction)
    contributors.append(ContributorResult(
        name="Preload force variation",
        source=f"±{tol.preload_variation_pct:.0f}% of {tol.preload_per_ball_N:.0f}N/ball",
        value_um=preload_var,
        xy_effect_um=0.0,
        z_effect_um=preload_var,
        category="repeatability",
        distribution="gaussian",
    ))

    # 5. Pin diameter tolerance — accuracy (systematic per cradle)
    # Pin diameter variation changes the cradle valley depth.
    # For a two-pin cradle, ball center height above pin axis plane:
    #   h = sqrt((ball_r + pin_r)^2 - (pin_center_dist/2)^2)
    # Differentiating wrt pin_r:
    #   dh/d(pin_r) = (ball_r + pin_r) / h - 0 (pin_center_dist is fixed by holes)
    # Actually, if pins are in fixed holes and pin diameter changes:
    #   the effective V-angle changes, shifting ball center radially and vertically.
    # Simplified: delta_z ≈ delta_pin_r / tan(alpha)
    # where alpha = arctan(ball_r / (pin_center_dist/2))
    half_pin_sep = geo.pin_center_dist_mm / 2.0
    alpha = math.atan2(ball_r, half_pin_sep)  # half-angle of cradle
    pin_r_tol_um = pin_tol_range_um / 2.0  # ± from midpoint
    pin_z_shift = pin_r_tol_um / math.tan(alpha) if alpha > 0 else 0
    pin_xy_shift = pin_r_tol_um * 0.5  # lateral shift is smaller
    contributors.append(ContributorResult(
        name="Pin diameter (h6)",
        source=f"{geo.pin_dia_mm:.0f}mm h6: {pin_lower_um:+d}/{pin_upper_um:+d} um",
        value_um=pin_tol_range_um,
        xy_effect_um=pin_xy_shift,
        z_effect_um=pin_z_shift,
        category="accuracy",
        distribution="systematic",
    ))

    # 6. Ball diameter tolerance — accuracy (lot-to-lot)
    # If all 3 balls are from the same lot, diameter error is systematic:
    # all balls shift the same amount. This shifts Z uniformly (accuracy, not repeat).
    # Z shift: ball_dia_tol / (2 * sin(alpha)) where alpha is cradle half-angle
    ball_z_shift = ball_dia_tol / (2 * math.sin(alpha)) if alpha > 0 else 0
    contributors.append(ContributorResult(
        name="Ball diameter tolerance",
        source=f"ISO 3290 {tol.ball_grade}: ±{ball_dia_tol:.3f} um",
        value_um=ball_dia_tol,
        xy_effect_um=0.0,  # symmetric, no XY shift if all same
        z_effect_um=ball_z_shift,
        category="accuracy",
        distribution="systematic",
    ))

    # 7. Pin hole position — accuracy
    # Direct 1:1 geometric contribution. This is usually the dominant accuracy term.
    pos_tol_um = tol.machining_position_tol_mm * 1000  # convert mm to um
    # Each cradle has 2 pins; coupling has 3 cradles.
    # Position error at each cradle shifts that constraint point.
    # For 3-vee coupling, XY accuracy ≈ sqrt(2) * single_cradle_position_error
    # (geometric average over 3 cradle contributions to XY)
    pos_xy = pos_tol_um * math.sqrt(2)
    pos_z = pos_tol_um * 1.5  # Z is amplified by cradle geometry
    contributors.append(ContributorResult(
        name="Pin hole position (machining)",
        source=f"TP ±{tol.machining_position_tol_mm:.3f}mm = ±{pos_tol_um:.1f} um",
        value_um=pos_tol_um,
        xy_effect_um=pos_xy,
        z_effect_um=pos_z,
        category="accuracy",
        distribution="systematic",
    ))

    # 8. Ball pocket position — accuracy
    # Same magnitude as pin hole position (same machining process).
    pocket_xy = pos_tol_um * math.sqrt(2)
    pocket_z = pos_tol_um * 1.5
    contributors.append(ContributorResult(
        name="Ball pocket position (machining)",
        source=f"TP ±{tol.machining_position_tol_mm:.3f}mm = ±{pos_tol_um:.1f} um",
        value_um=pos_tol_um,
        xy_effect_um=pocket_xy,
        z_effect_um=pocket_z,
        category="accuracy",
        distribution="systematic",
    ))

    # --- Thermal sensitivity ---
    # Aluminum coupling, both halves expand symmetrically → center is stable.
    # If temperature differs between base and pallet: differential expansion.
    thermal_xy_per_degC = CTE_ALUMINUM * bcr * 1e3  # um/degC
    thermal_z_per_degC = CTE_ALUMINUM * geo.base_thick_mm * 1e3

    thermal_xy = thermal_xy_per_degC * tol.delta_T_degC
    thermal_z = thermal_z_per_degC * tol.delta_T_degC
    contributors.append(ContributorResult(
        name="Thermal expansion",
        source=f"Al 6061 CTE {CTE_ALUMINUM*1e6:.1f} um/m/°C, ΔT ±{tol.delta_T_degC:.0f}°C",
        value_um=thermal_xy,
        xy_effect_um=thermal_xy,
        z_effect_um=thermal_z,
        category="accuracy",
        distribution="systematic",
    ))

    # --- RSS repeatability ---
    repeat_contributors = [c for c in contributors if c.category == "repeatability"]
    rss_xy = math.sqrt(sum(c.xy_effect_um**2 for c in repeat_contributors))
    rss_z = math.sqrt(sum(c.z_effect_um**2 for c in repeat_contributors))

    # --- Worst-case accuracy ---
    acc_contributors = [c for c in contributors if c.category == "accuracy"]
    wc_xy = sum(c.xy_effect_um for c in acc_contributors)
    wc_z = sum(c.z_effect_um for c in acc_contributors)

    # --- RSS accuracy (more realistic than worst-case) ---
    rss_acc_xy = math.sqrt(sum(c.xy_effect_um**2 for c in acc_contributors))
    rss_acc_z = math.sqrt(sum(c.z_effect_um**2 for c in acc_contributors))

    return {
        "geometry": geo,
        "tolerances": tol,
        "hertzian": {
            "contact_half_width_mm": contact_a,
            "max_pressure_MPa": contact_p,
            "approach_um": contact_delta,
            "preload_per_ball_N": tol.preload_per_ball_N,
        },
        "contributors": contributors,
        "repeatability_xy_um": rss_xy,
        "repeatability_z_um": rss_z,
        "accuracy_wc_xy_um": wc_xy,
        "accuracy_wc_z_um": wc_z,
        "accuracy_rss_xy_um": rss_acc_xy,
        "accuracy_rss_z_um": rss_acc_z,
        "thermal_sensitivity_xy_um_per_degC": thermal_xy_per_degC,
        "thermal_sensitivity_z_um_per_degC": thermal_z_per_degC,
    }


# ---------------------------------------------------------------------------
# Report generation (Markdown)
# ---------------------------------------------------------------------------

def format_report(results: dict) -> str:
    """Generate a markdown tolerance stack-up report."""
    geo = results["geometry"]
    tol = results["tolerances"]
    hz = results["hertzian"]
    contribs = results["contributors"]

    lines = []
    lines.append(f"# Tolerance Stack-Up Report: {geo.label}")
    lines.append("")
    lines.append(f"## Coupling Geometry ({geo.plate_size_mm:.0f}mm plate)")
    lines.append("")
    lines.append(f"| Parameter | Value |")
    lines.append(f"|---|---|")
    lines.append(f"| Plate size | {geo.plate_size_mm:.0f} x {geo.plate_size_mm:.0f} mm |")
    lines.append(f"| Ball diameter | {geo.ball_dia_mm:.1f} mm ({tol.ball_grade}) |")
    lines.append(f"| Pin diameter | {geo.pin_dia_mm:.1f} mm (h6) |")
    lines.append(f"| Bolt circle radius | {geo.bolt_circle_r_mm:.1f} mm |")
    lines.append(f"| Pin center distance | {geo.pin_center_dist_mm:.2f} mm |")
    lines.append(f"| Base thickness | {geo.base_thick_mm:.1f} mm |")
    lines.append(f"| Pallet thickness | {geo.pallet_thick_mm:.1f} mm |")
    lines.append(f"| Preload per ball | {tol.preload_per_ball_N:.0f} N |")
    lines.append("")

    lines.append("## Hertzian Contact Analysis")
    lines.append("")
    lines.append("Ball-on-cylinder (convex-convex), per contact point:")
    lines.append("")
    lines.append(f"| Parameter | Value |")
    lines.append(f"|---|---|")
    lines.append(f"| Contact semi-width | {hz['contact_half_width_mm']*1000:.1f} um |")
    lines.append(f"| Peak pressure | {hz['max_pressure_MPa']:.0f} MPa |")
    lines.append(f"| Elastic approach | {hz['approach_um']:.3f} um |")
    lines.append("")
    lines.append(f"The small contact patch ({hz['contact_half_width_mm']*1000:.0f} um) is what "
                 f"makes kinematic couplings deterministic: the contact centroid position "
                 f"depends only on the macro-geometry of the ball and pin, not on "
                 f"surface conformity over a large area.")
    lines.append("")

    lines.append("## Repeatability Budget (RSS)")
    lines.append("")
    lines.append("Repeatability = cycle-to-cycle variation when coupling is repeatedly "
                 "engaged and disengaged. Only RANDOM contributors participate.")
    lines.append("")
    lines.append("| Contributor | Source | XY (um) | Z (um) |")
    lines.append("|---|---|---|---|")
    for c in contribs:
        if c.category == "repeatability":
            lines.append(f"| {c.name} | {c.source} | {c.xy_effect_um:.3f} | "
                         f"{c.z_effect_um:.3f} |")
    lines.append(f"| **RSS Total** | | "
                 f"**{results['repeatability_xy_um']:.3f}** | "
                 f"**{results['repeatability_z_um']:.3f}** |")
    lines.append("")
    lines.append(f"**Predicted 1-sigma repeatability: "
                 f"XY = {results['repeatability_xy_um']:.2f} um, "
                 f"Z = {results['repeatability_z_um']:.2f} um**")
    lines.append("")
    lines.append(f"At 3-sigma (99.7% confidence): "
                 f"XY < {results['repeatability_xy_um']*3:.1f} um, "
                 f"Z < {results['repeatability_z_um']*3:.1f} um")
    lines.append("")

    lines.append("## Accuracy Budget")
    lines.append("")
    lines.append("Accuracy = deviation from nominal position. SYSTEMATIC contributors "
                 "create a fixed offset that does not vary cycle-to-cycle.")
    lines.append("")
    lines.append("| Contributor | Source | XY (um) | Z (um) |")
    lines.append("|---|---|---|---|")
    for c in contribs:
        if c.category == "accuracy":
            lines.append(f"| {c.name} | {c.source} | {c.xy_effect_um:.1f} | "
                         f"{c.z_effect_um:.1f} |")
    lines.append(f"| **Worst-case sum** | | "
                 f"**{results['accuracy_wc_xy_um']:.1f}** | "
                 f"**{results['accuracy_wc_z_um']:.1f}** |")
    lines.append(f"| **RSS (statistical)** | | "
                 f"**{results['accuracy_rss_xy_um']:.1f}** | "
                 f"**{results['accuracy_rss_z_um']:.1f}** |")
    lines.append("")

    lines.append("## Thermal Sensitivity")
    lines.append("")
    lines.append(f"| Axis | Drift per °C | Drift at ±{tol.delta_T_degC:.0f}°C |")
    lines.append(f"|---|---|---|")
    lines.append(f"| XY | {results['thermal_sensitivity_xy_um_per_degC']:.2f} um/°C | "
                 f"±{results['thermal_sensitivity_xy_um_per_degC']*tol.delta_T_degC:.1f} um |")
    lines.append(f"| Z | {results['thermal_sensitivity_z_um_per_degC']:.2f} um/°C | "
                 f"±{results['thermal_sensitivity_z_um_per_degC']*tol.delta_T_degC:.1f} um |")
    lines.append("")
    lines.append("Note: If both coupling halves are the same material (aluminum) and at the "
                 "same temperature, the coupling center is thermally stable (symmetric "
                 "expansion). Thermal drift only matters for differential temperature "
                 "between base and pallet.")
    lines.append("")

    lines.append("## Dominant Contributors")
    lines.append("")

    # Find dominant repeatability contributor
    rep = [c for c in contribs if c.category == "repeatability"]
    dom_rep = max(rep, key=lambda c: c.xy_effect_um)
    lines.append(f"- **Repeatability** dominated by: {dom_rep.name} "
                 f"({dom_rep.xy_effect_um:.3f} um XY)")

    # Find dominant accuracy contributor
    acc = [c for c in contribs if c.category == "accuracy"]
    dom_acc = max(acc, key=lambda c: c.xy_effect_um)
    lines.append(f"- **Accuracy** dominated by: {dom_acc.name} "
                 f"({dom_acc.xy_effect_um:.1f} um XY)")
    lines.append("")
    lines.append("The coupling hardware (balls and pins) determines repeatability. "
                 "The machining determines accuracy. You buy accuracy with machine time; "
                 "you get repeatability essentially for free from the kinematic constraint.")
    lines.append("")

    lines.append("## GD&T Recommendations")
    lines.append("")
    lines.append("Based on this analysis, the following GD&T callouts are recommended:")
    lines.append("")
    lines.append("### Base Station")
    lines.append(f"- Top face (Datum A): Flatness {geo.base_thick_mm * 0.001:.3f}mm "
                 f"({geo.base_thick_mm * 1:.0f} um)")
    lines.append(f"- Pin press-fit holes: True Position "
                 f"dia {tol.machining_position_tol_mm*2:.3f}mm "
                 f"| A | B | C |")
    lines.append(f"- Pin hole diameter: "
                 f"{geo.pin_dia_mm * 0.994:.3f} +0.000/-{geo.pin_dia_mm * 0.006:.3f}mm "
                 f"(interference fit for h6 pins)")
    lines.append(f"- Receiver bore: True Position "
                 f"dia 0.050mm | A | B |")
    lines.append(f"- Receiver slot positions: True Position dia 0.100mm | A | B | "
                 f"(less critical, tab clearance only)")
    lines.append("")
    lines.append("### Pallet")
    lines.append(f"- Bottom face (Datum A): Flatness {geo.pallet_thick_mm * 0.001:.3f}mm")
    lines.append(f"- Ball pocket holes: True Position "
                 f"dia {tol.machining_position_tol_mm*2:.3f}mm | A | B | C |")
    lines.append(f"- Ball pocket diameter: "
                 f"{geo.ball_dia_mm * 0.994:.3f} +0.000/-{geo.ball_dia_mm * 0.006:.3f}mm "
                 f"(light press for balls)")
    lines.append(f"- Latch pin axis: Concentricity dia 0.050mm | A | B |")
    lines.append("")

    return "\n".join(lines)


def format_comparison_table(all_results: List[dict]) -> str:
    """Generate a comparison table across multiple presets."""
    lines = []
    lines.append("# Multi-Size Comparison")
    lines.append("")
    lines.append("| Parameter | " + " | ".join(
        r["geometry"].label for r in all_results) + " |")
    lines.append("|---|" + "|".join("---" for _ in all_results) + "|")

    rows = [
        ("Plate size (mm)", lambda r: f"{r['geometry'].plate_size_mm:.0f}"),
        ("Ball dia (mm)", lambda r: f"{r['geometry'].ball_dia_mm:.1f}"),
        ("Pin dia (mm)", lambda r: f"{r['geometry'].pin_dia_mm:.1f}"),
        ("Bolt circle R (mm)", lambda r: f"{r['geometry'].bolt_circle_r_mm:.1f}"),
        ("Contact patch (um)", lambda r: f"{r['hertzian']['contact_half_width_mm']*1000:.1f}"),
        ("Peak pressure (MPa)", lambda r: f"{r['hertzian']['max_pressure_MPa']:.0f}"),
        ("**Repeatability XY (um)**",
         lambda r: f"**{r['repeatability_xy_um']:.2f}**"),
        ("**Repeatability Z (um)**",
         lambda r: f"**{r['repeatability_z_um']:.2f}**"),
        ("Repeat XY 3-sigma (um)",
         lambda r: f"{r['repeatability_xy_um']*3:.1f}"),
        ("Accuracy RSS XY (um)",
         lambda r: f"{r['accuracy_rss_xy_um']:.1f}"),
        ("Accuracy RSS Z (um)",
         lambda r: f"{r['accuracy_rss_z_um']:.1f}"),
        ("Thermal XY (um/°C)",
         lambda r: f"{r['thermal_sensitivity_xy_um_per_degC']:.2f}"),
    ]

    for label, fn in rows:
        lines.append(f"| {label} | " + " | ".join(fn(r) for r in all_results) + " |")

    lines.append("")
    lines.append("Key finding: repeatability is nearly identical across sizes because it "
                 "is dominated by ball sphericity and surface finish, which are grade-specified "
                 "and do not change with coupling size. Accuracy scales with machining "
                 "tolerances, which are proportional to feature count and travel distances.")
    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Tolerance stack-up analysis for kinematic pallet coupling"
    )
    parser.add_argument("--preset", choices=list(GEOMETRY_PRESETS.keys()),
                        default="medium", help="Size preset (default: medium)")
    parser.add_argument("--all", action="store_true",
                        help="Analyze and compare all presets")
    parser.add_argument("--output", type=str, default=None,
                        help="Output markdown file path")
    parser.add_argument("--ball-grade", default="G25",
                        choices=list(BALL_GRADES.keys()),
                        help="Ball bearing grade (default: G25)")
    parser.add_argument("--machining-tol", type=float, default=0.025,
                        help="Machining position tolerance in mm (default: 0.025)")
    parser.add_argument("--delta-t", type=float, default=5.0,
                        help="Temperature swing in degC (default: 5.0)")
    args = parser.parse_args()

    # Default output path
    if args.output is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.output = os.path.join(script_dir, "tolerance_stackup_report.md")

    tol = ToleranceInputs(
        ball_grade=args.ball_grade,
        machining_position_tol_mm=args.machining_tol,
        delta_T_degC=args.delta_t,
    )

    if args.all:
        all_results = []
        report_parts = []
        for name in ["small", "medium", "large", "xl"]:
            geo = GEOMETRY_PRESETS[name]
            result = analyze_coupling(geo, tol)
            all_results.append(result)
            report_parts.append(format_report(result))

        comparison = format_comparison_table(all_results)
        full_report = comparison + "\n---\n\n" + "\n\n---\n\n".join(report_parts)
    else:
        geo = GEOMETRY_PRESETS[args.preset]
        result = analyze_coupling(geo, tol)
        full_report = format_report(result)

    # Write report
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(full_report)
    print(f"Report written to: {args.output}")

    # Print summary to console
    if args.all:
        for r in all_results:
            g = r["geometry"]
            print(f"  {g.label:8s}: Repeat XY={r['repeatability_xy_um']:.2f}um  "
                  f"Z={r['repeatability_z_um']:.2f}um  |  "
                  f"Accuracy XY={r['accuracy_rss_xy_um']:.1f}um  "
                  f"Z={r['accuracy_rss_z_um']:.1f}um")
    else:
        print(f"  Repeatability:  XY = {result['repeatability_xy_um']:.2f} um  "
              f"(3σ: {result['repeatability_xy_um']*3:.1f} um)")
        print(f"                  Z  = {result['repeatability_z_um']:.2f} um  "
              f"(3σ: {result['repeatability_z_um']*3:.1f} um)")
        print(f"  Accuracy (RSS): XY = {result['accuracy_rss_xy_um']:.1f} um")
        print(f"                  Z  = {result['accuracy_rss_z_um']:.1f} um")


if __name__ == "__main__":
    main()
