# Multi-Size Comparison

| Parameter | small | medium | large | xl |
|---|---|---|---|---|
| Plate size (mm) | 80 | 120 | 200 | 300 |
| Ball dia (mm) | 3.0 | 5.0 | 8.0 | 10.0 |
| Pin dia (mm) | 3.0 | 5.0 | 8.0 | 10.0 |
| Bolt circle R (mm) | 23.4 | 35.0 | 58.4 | 87.6 |
| Contact patch (um) | 36.6 | 43.4 | 50.8 | 54.7 |
| Peak pressure (MPa) | 3562 | 2534 | 1853 | 1596 |
| **Repeatability XY (um)** | **0.43** | **0.43** | **0.43** | **0.43** |
| **Repeatability Z (um)** | **0.57** | **0.57** | **0.57** | **0.57** |
| Repeat XY 3-sigma (um) | 1.3 | 1.3 | 1.3 | 1.3 |
| Accuracy RSS XY (um) | 50.1 | 50.2 | 50.5 | 51.1 |
| Accuracy RSS Z (um) | 53.2 | 53.4 | 53.6 | 53.7 |
| Thermal XY (um/°C) | 0.54 | 0.81 | 1.35 | 2.02 |

Key finding: repeatability is nearly identical across sizes because it is dominated by ball sphericity and surface finish, which are grade-specified and do not change with coupling size. Accuracy scales with machining tolerances, which are proportional to feature count and travel distances.

---

# Tolerance Stack-Up Report: small

## Coupling Geometry (80mm plate)

| Parameter | Value |
|---|---|
| Plate size | 80 x 80 mm |
| Ball diameter | 3.0 mm (G25) |
| Pin diameter | 3.0 mm (h6) |
| Bolt circle radius | 23.4 mm |
| Pin center distance | 4.24 mm |
| Base thickness | 13.4 mm |
| Pallet thickness | 6.6 mm |
| Preload per ball | 10 N |

## Hertzian Contact Analysis

Ball-on-cylinder (convex-convex), per contact point:

| Parameter | Value |
|---|---|
| Contact semi-width | 36.6 um |
| Peak pressure | 3562 MPa |
| Elastic approach | 1.787 um |

The small contact patch (37 um) is what makes kinematic couplings deterministic: the contact centroid position depends only on the macro-geometry of the ball and pin, not on surface conformity over a large area.

## Repeatability Budget (RSS)

Repeatability = cycle-to-cycle variation when coupling is repeatedly engaged and disengaged. Only RANDOM contributors participate.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Ball sphericity | ISO 3290 G25: 0.625 um | 0.207 | 0.271 |
| Surface finish scatter | Ball Ra 0.020 um + Pin Ra 0.4 um | 0.189 | 0.231 |
| Contamination allowance | 0.5 um effective Ra (clean shop) | 0.332 | 0.433 |
| Preload force variation | ±10% of 10N/ball | 0.000 | 0.119 |
| **RSS Total** | | **0.435** | **0.573** |

**Predicted 1-sigma repeatability: XY = 0.43 um, Z = 0.57 um**

At 3-sigma (99.7% confidence): XY < 1.3 um, Z < 1.7 um

## Accuracy Budget

Accuracy = deviation from nominal position. SYSTEMATIC contributors create a fixed offset that does not vary cycle-to-cycle.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Pin diameter (h6) | 3mm h6: -6/+0 um | 1.5 | 4.2 |
| Ball diameter tolerance | ISO 3290 G25: ±0.625 um | 0.0 | 0.5 |
| Pin hole position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Ball pocket position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Thermal expansion | Al 6061 CTE 23.1 um/m/°C, ΔT ±5°C | 2.7 | 1.5 |
| **Worst-case sum** | | **74.9** | **81.3** |
| **RSS (statistical)** | | **50.1** | **53.2** |

## Thermal Sensitivity

| Axis | Drift per °C | Drift at ±5°C |
|---|---|---|
| XY | 0.54 um/°C | ±2.7 um |
| Z | 0.31 um/°C | ±1.5 um |

Note: If both coupling halves are the same material (aluminum) and at the same temperature, the coupling center is thermally stable (symmetric expansion). Thermal drift only matters for differential temperature between base and pallet.

## Dominant Contributors

- **Repeatability** dominated by: Contamination allowance (0.332 um XY)
- **Accuracy** dominated by: Pin hole position (machining) (35.4 um XY)

The coupling hardware (balls and pins) determines repeatability. The machining determines accuracy. You buy accuracy with machine time; you get repeatability essentially for free from the kinematic constraint.

## GD&T Recommendations

Based on this analysis, the following GD&T callouts are recommended:

### Base Station
- Top face (Datum A): Flatness 0.013mm (13 um)
- Pin press-fit holes: True Position dia 0.050mm | A | B | C |
- Pin hole diameter: 2.982 +0.000/-0.018mm (interference fit for h6 pins)
- Receiver bore: True Position dia 0.050mm | A | B |
- Receiver slot positions: True Position dia 0.100mm | A | B | (less critical, tab clearance only)

### Pallet
- Bottom face (Datum A): Flatness 0.007mm
- Ball pocket holes: True Position dia 0.050mm | A | B | C |
- Ball pocket diameter: 2.982 +0.000/-0.018mm (light press for balls)
- Latch pin axis: Concentricity dia 0.050mm | A | B |


---

# Tolerance Stack-Up Report: medium

## Coupling Geometry (120mm plate)

| Parameter | Value |
|---|---|
| Plate size | 120 x 120 mm |
| Ball diameter | 5.0 mm (G25) |
| Pin diameter | 5.0 mm (h6) |
| Bolt circle radius | 35.0 mm |
| Pin center distance | 7.07 mm |
| Base thickness | 20.0 mm |
| Pallet thickness | 10.0 mm |
| Preload per ball | 10 N |

## Hertzian Contact Analysis

Ball-on-cylinder (convex-convex), per contact point:

| Parameter | Value |
|---|---|
| Contact semi-width | 43.4 um |
| Peak pressure | 2534 MPa |
| Elastic approach | 1.507 um |

The small contact patch (43 um) is what makes kinematic couplings deterministic: the contact centroid position depends only on the macro-geometry of the ball and pin, not on surface conformity over a large area.

## Repeatability Budget (RSS)

Repeatability = cycle-to-cycle variation when coupling is repeatedly engaged and disengaged. Only RANDOM contributors participate.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Ball sphericity | ISO 3290 G25: 0.625 um | 0.207 | 0.271 |
| Surface finish scatter | Ball Ra 0.020 um + Pin Ra 0.4 um | 0.189 | 0.231 |
| Contamination allowance | 0.5 um effective Ra (clean shop) | 0.332 | 0.433 |
| Preload force variation | ±10% of 10N/ball | 0.000 | 0.100 |
| **RSS Total** | | **0.435** | **0.569** |

**Predicted 1-sigma repeatability: XY = 0.43 um, Z = 0.57 um**

At 3-sigma (99.7% confidence): XY < 1.3 um, Z < 1.7 um

## Accuracy Budget

Accuracy = deviation from nominal position. SYSTEMATIC contributors create a fixed offset that does not vary cycle-to-cycle.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Pin diameter (h6) | 5mm h6: -8/+0 um | 2.0 | 5.7 |
| Ball diameter tolerance | ISO 3290 G25: ±0.625 um | 0.0 | 0.5 |
| Pin hole position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Ball pocket position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Thermal expansion | Al 6061 CTE 23.1 um/m/°C, ΔT ±5°C | 4.0 | 2.3 |
| **Worst-case sum** | | **76.8** | **83.5** |
| **RSS (statistical)** | | **50.2** | **53.4** |

## Thermal Sensitivity

| Axis | Drift per °C | Drift at ±5°C |
|---|---|---|
| XY | 0.81 um/°C | ±4.0 um |
| Z | 0.46 um/°C | ±2.3 um |

Note: If both coupling halves are the same material (aluminum) and at the same temperature, the coupling center is thermally stable (symmetric expansion). Thermal drift only matters for differential temperature between base and pallet.

## Dominant Contributors

- **Repeatability** dominated by: Contamination allowance (0.332 um XY)
- **Accuracy** dominated by: Pin hole position (machining) (35.4 um XY)

The coupling hardware (balls and pins) determines repeatability. The machining determines accuracy. You buy accuracy with machine time; you get repeatability essentially for free from the kinematic constraint.

## GD&T Recommendations

Based on this analysis, the following GD&T callouts are recommended:

### Base Station
- Top face (Datum A): Flatness 0.020mm (20 um)
- Pin press-fit holes: True Position dia 0.050mm | A | B | C |
- Pin hole diameter: 4.970 +0.000/-0.030mm (interference fit for h6 pins)
- Receiver bore: True Position dia 0.050mm | A | B |
- Receiver slot positions: True Position dia 0.100mm | A | B | (less critical, tab clearance only)

### Pallet
- Bottom face (Datum A): Flatness 0.010mm
- Ball pocket holes: True Position dia 0.050mm | A | B | C |
- Ball pocket diameter: 4.970 +0.000/-0.030mm (light press for balls)
- Latch pin axis: Concentricity dia 0.050mm | A | B |


---

# Tolerance Stack-Up Report: large

## Coupling Geometry (200mm plate)

| Parameter | Value |
|---|---|
| Plate size | 200 x 200 mm |
| Ball diameter | 8.0 mm (G25) |
| Pin diameter | 8.0 mm (h6) |
| Bolt circle radius | 58.4 mm |
| Pin center distance | 11.31 mm |
| Base thickness | 33.4 mm |
| Pallet thickness | 16.6 mm |
| Preload per ball | 10 N |

## Hertzian Contact Analysis

Ball-on-cylinder (convex-convex), per contact point:

| Parameter | Value |
|---|---|
| Contact semi-width | 50.8 um |
| Peak pressure | 1853 MPa |
| Elastic approach | 1.289 um |

The small contact patch (51 um) is what makes kinematic couplings deterministic: the contact centroid position depends only on the macro-geometry of the ball and pin, not on surface conformity over a large area.

## Repeatability Budget (RSS)

Repeatability = cycle-to-cycle variation when coupling is repeatedly engaged and disengaged. Only RANDOM contributors participate.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Ball sphericity | ISO 3290 G25: 0.625 um | 0.207 | 0.271 |
| Surface finish scatter | Ball Ra 0.020 um + Pin Ra 0.4 um | 0.189 | 0.231 |
| Contamination allowance | 0.5 um effective Ra (clean shop) | 0.332 | 0.433 |
| Preload force variation | ±10% of 10N/ball | 0.000 | 0.086 |
| **RSS Total** | | **0.435** | **0.567** |

**Predicted 1-sigma repeatability: XY = 0.43 um, Z = 0.57 um**

At 3-sigma (99.7% confidence): XY < 1.3 um, Z < 1.7 um

## Accuracy Budget

Accuracy = deviation from nominal position. SYSTEMATIC contributors create a fixed offset that does not vary cycle-to-cycle.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Pin diameter (h6) | 8mm h6: -9/+0 um | 2.2 | 6.4 |
| Ball diameter tolerance | ISO 3290 G25: ±0.625 um | 0.0 | 0.5 |
| Pin hole position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Ball pocket position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Thermal expansion | Al 6061 CTE 23.1 um/m/°C, ΔT ±5°C | 6.7 | 3.9 |
| **Worst-case sum** | | **79.7** | **85.8** |
| **RSS (statistical)** | | **50.5** | **53.6** |

## Thermal Sensitivity

| Axis | Drift per °C | Drift at ±5°C |
|---|---|---|
| XY | 1.35 um/°C | ±6.7 um |
| Z | 0.77 um/°C | ±3.9 um |

Note: If both coupling halves are the same material (aluminum) and at the same temperature, the coupling center is thermally stable (symmetric expansion). Thermal drift only matters for differential temperature between base and pallet.

## Dominant Contributors

- **Repeatability** dominated by: Contamination allowance (0.332 um XY)
- **Accuracy** dominated by: Pin hole position (machining) (35.4 um XY)

The coupling hardware (balls and pins) determines repeatability. The machining determines accuracy. You buy accuracy with machine time; you get repeatability essentially for free from the kinematic constraint.

## GD&T Recommendations

Based on this analysis, the following GD&T callouts are recommended:

### Base Station
- Top face (Datum A): Flatness 0.033mm (33 um)
- Pin press-fit holes: True Position dia 0.050mm | A | B | C |
- Pin hole diameter: 7.952 +0.000/-0.048mm (interference fit for h6 pins)
- Receiver bore: True Position dia 0.050mm | A | B |
- Receiver slot positions: True Position dia 0.100mm | A | B | (less critical, tab clearance only)

### Pallet
- Bottom face (Datum A): Flatness 0.017mm
- Ball pocket holes: True Position dia 0.050mm | A | B | C |
- Ball pocket diameter: 7.952 +0.000/-0.048mm (light press for balls)
- Latch pin axis: Concentricity dia 0.050mm | A | B |


---

# Tolerance Stack-Up Report: xl

## Coupling Geometry (300mm plate)

| Parameter | Value |
|---|---|
| Plate size | 300 x 300 mm |
| Ball diameter | 10.0 mm (G25) |
| Pin diameter | 10.0 mm (h6) |
| Bolt circle radius | 87.6 mm |
| Pin center distance | 14.14 mm |
| Base thickness | 50.1 mm |
| Pallet thickness | 24.9 mm |
| Preload per ball | 10 N |

## Hertzian Contact Analysis

Ball-on-cylinder (convex-convex), per contact point:

| Parameter | Value |
|---|---|
| Contact semi-width | 54.7 um |
| Peak pressure | 1596 MPa |
| Elastic approach | 1.196 um |

The small contact patch (55 um) is what makes kinematic couplings deterministic: the contact centroid position depends only on the macro-geometry of the ball and pin, not on surface conformity over a large area.

## Repeatability Budget (RSS)

Repeatability = cycle-to-cycle variation when coupling is repeatedly engaged and disengaged. Only RANDOM contributors participate.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Ball sphericity | ISO 3290 G25: 0.625 um | 0.207 | 0.271 |
| Surface finish scatter | Ball Ra 0.020 um + Pin Ra 0.4 um | 0.189 | 0.231 |
| Contamination allowance | 0.5 um effective Ra (clean shop) | 0.332 | 0.433 |
| Preload force variation | ±10% of 10N/ball | 0.000 | 0.080 |
| **RSS Total** | | **0.435** | **0.566** |

**Predicted 1-sigma repeatability: XY = 0.43 um, Z = 0.57 um**

At 3-sigma (99.7% confidence): XY < 1.3 um, Z < 1.7 um

## Accuracy Budget

Accuracy = deviation from nominal position. SYSTEMATIC contributors create a fixed offset that does not vary cycle-to-cycle.

| Contributor | Source | XY (um) | Z (um) |
|---|---|---|---|
| Pin diameter (h6) | 10mm h6: -9/+0 um | 2.2 | 6.4 |
| Ball diameter tolerance | ISO 3290 G25: ±0.625 um | 0.0 | 0.5 |
| Pin hole position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Ball pocket position (machining) | TP ±0.025mm = ±25.0 um | 35.4 | 37.5 |
| Thermal expansion | Al 6061 CTE 23.1 um/m/°C, ΔT ±5°C | 10.1 | 5.8 |
| **Worst-case sum** | | **83.1** | **87.7** |
| **RSS (statistical)** | | **51.1** | **53.7** |

## Thermal Sensitivity

| Axis | Drift per °C | Drift at ±5°C |
|---|---|---|
| XY | 2.02 um/°C | ±10.1 um |
| Z | 1.16 um/°C | ±5.8 um |

Note: If both coupling halves are the same material (aluminum) and at the same temperature, the coupling center is thermally stable (symmetric expansion). Thermal drift only matters for differential temperature between base and pallet.

## Dominant Contributors

- **Repeatability** dominated by: Contamination allowance (0.332 um XY)
- **Accuracy** dominated by: Pin hole position (machining) (35.4 um XY)

The coupling hardware (balls and pins) determines repeatability. The machining determines accuracy. You buy accuracy with machine time; you get repeatability essentially for free from the kinematic constraint.

## GD&T Recommendations

Based on this analysis, the following GD&T callouts are recommended:

### Base Station
- Top face (Datum A): Flatness 0.050mm (50 um)
- Pin press-fit holes: True Position dia 0.050mm | A | B | C |
- Pin hole diameter: 9.940 +0.000/-0.060mm (interference fit for h6 pins)
- Receiver bore: True Position dia 0.050mm | A | B |
- Receiver slot positions: True Position dia 0.100mm | A | B | (less critical, tab clearance only)

### Pallet
- Bottom face (Datum A): Flatness 0.025mm
- Ball pocket holes: True Position dia 0.050mm | A | B | C |
- Ball pocket diameter: 9.940 +0.000/-0.060mm (light press for balls)
- Latch pin axis: Concentricity dia 0.050mm | A | B |
