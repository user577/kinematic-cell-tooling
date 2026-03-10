# Design Notes — Kinematic Cell Tooling Platform

## 1. Kinematic Coupling Theory

### Exact Constraint Principle

A rigid body in 3D space has six degrees of freedom (DOF): three
translational (X, Y, Z) and three rotational (Rx, Ry, Rz). A fixture
that constrains exactly six DOF — no more, no fewer — is called an
exact-constraint fixture. Under-constraint leaves residual motion.
Over-constraint creates indeterminate stress states that fight
manufacturing tolerances and cause hysteresis.

The critical insight: **an over-constrained fixture is not more rigid,
it is less repeatable.** Two competing constraints on the same DOF
produce a winner based on whichever surface has more local stiffness
at that moment, which varies with temperature, bolt torque, and wear.
Repeatability requires determinism, and determinism requires that each
DOF be constrained exactly once.

### Kelvin Coupling Geometry

The Kelvin coupling (also called Maxwell three-groove coupling) uses
three contact pairs arranged 120 degrees apart on a bolt circle:

- **Trihedral socket (or three-pin nest):** One ball seats against three
  flat surfaces. Constrains 3 DOF (X, Y, Z).
- **V-groove (or two-pin cradle):** One ball seats against two cylindrical
  surfaces. Constrains 2 DOF (one translation, one rotation).
- **Flat (or single-pin contact):** One ball seats against a single flat
  surface. Constrains 1 DOF (Z at that location).

Total: 3 + 2 + 1 = 6 DOF. Exactly constrained.

In this platform we use the **three-vee variant** (all three contacts are
two-pin cradles), which constrains 2 + 2 + 2 = 6 DOF when the cradles
are oriented tangentially to the bolt circle. This is mechanically simpler
than the Kelvin arrangement (no trihedral socket to machine) and achieves
equivalent repeatability when the cradle orientations are correct.

### Ball-on-Cylinder vs Ball-on-Groove Contact

A precision ball bearing resting on two ground dowel pins creates a
Hertzian line contact along each pin. The contact geometry is:

- **Ball-on-cylinder (convex-convex):** Small contact patch, high stress,
  extremely repeatable because the contact centroid is deterministic.
  This is what we use.
- **Ball-in-groove (convex-concave):** Larger contact patch, lower stress,
  but the contact centroid depends on ball and groove diameters matching.
  Any diameter variation shifts the contact point and degrades repeatability.

Ground dowel pins (h6 tolerance) and precision ball bearings (G25 grade)
are commodity items. A two-pin cradle costs under $3 in hardware.

### Repeatability Budget

With 5mm G25 balls (sphericity < 0.6 um) on h6 ground pins
(diameter tolerance +/-2.5 um), Hertzian contact theory predicts:

- Contact patch semi-width: ~15 um at 10N preload per ball
- Contact centroid uncertainty: < 1 um (dominated by surface finish)
- Full coupling repeatability: sub-5 um in XY, sub-10 um in Z
- Thermal drift: ~0.3 um/degC (aluminum, 35mm bolt circle)

This exceeds the requirements for robot cell work where the robot
itself has +/-0.02-0.05mm repeatability.

## 2. Why Incumbents Use Ball-Race Clamping (and Why It's Wrong)

Commercial tool changers (ATI, Schunk, Zimmer) use a common mechanism:
a set of steel balls pressed radially outward by a piston into a
circumferential groove in the tool plate. This creates a ball-race
clamp that provides both location and retention in a single mechanism.

**Advantages of ball-race clamping:**
- High retention force (hundreds of N)
- Compact axial envelope
- Well-understood, mature technology

**Why it degrades repeatability:**
- Location and retention are coupled. The balls must slide circumferentially
  as they are driven outward, creating friction hysteresis.
- The groove must be oversized to allow ball entry, so location depends on
  the piston force producing consistent radial displacement.
- Wear on ball or groove changes the seating geometry.
- Temperature changes piston force (spring rate, friction).

**The correct separation of concerns:**
- Use a **kinematic coupling** for location (deterministic, zero-friction
  nesting into exact-constraint contacts).
- Use a **separate mechanism** for retention (clamp, latch, magnet —
  anything that applies force along the nesting direction without
  competing with the location geometry).

This separation is why a $15 kinematic coupling outperforms a $2000
tool changer on repeatability: the geometry is deterministic regardless
of clamp force variation.

## 3. Twist-Lock Latch Rationale

### Why Gravity Retention Fails

A kinematic coupling preloaded only by gravity (pallet weight + workpiece
weight) has a critical vulnerability: **any force with a horizontal
component can tip or slide the pallet off the coupling.**

Failure modes under gravity-only retention:
- Robot approach vibration lifts the light edge of the pallet.
- Process forces (drilling, pressing) overcome coupling preload.
- Accidental contact knocks pallet off coupling.
- Acceleration during conveyor transport unseats the coupling.

Magnetic preload partially addresses this but has its own problems:
- Magnetic force drops as cube of distance, so a 1mm unseating
  reduces retention force by ~87%.
- Lateral magnetic stiffness is near zero.
- Cannot be switched off without electromagnets (adds complexity).

### Why Axial Retention is the Correct Separation of Concerns

The twist-lock latch provides **axial retention only** — it prevents the
pallet from lifting off the base station. It does not provide any
lateral location. This is exactly the correct separation:

- **Kinematic coupling** handles XY + theta location (repeatability).
- **Twist-lock latch** handles Z retention (security).
- **Gravity + optional spring** provides nesting preload (seating force).

The twist-lock is superior to a clamp or toggle because:
- It is self-engaging via a helical cam surface (no actuator needed to lock).
- A solenoid is needed only to release (fail-safe: power loss = locked).
- The cam geometry converts vertical descent into rotary locking motion.
- Retention is positive (mechanical interference, not friction).
- No side loads are introduced during engagement or release.

### Cam Drive Geometry

The receiver inner wall contains a helical cam track. As the pallet
descends, the latch pin tabs ride in the cam track and the pin
rotates 45 degrees over 15mm of Z travel. At the bottom of travel,
the tabs are behind the receiver slot lips and cannot withdraw.

To release: a 24VDC rotary solenoid applies 45 degrees of counter-
rotation, aligning tabs with slots, and a light spring lifts the
pallet 2-3mm to clear the cam engagement zone.

## 4. Electromagnetic Preload Theory

### Force vs. Current Relationship

An electromagnet pressing a ferromagnetic keeper against a kinematic
coupling produces preload force:

    F = (B^2 * A) / (2 * mu_0)

Where B = mu_0 * N * I / g (in the unsaturated regime), so:

    F proportional to I^2 (at constant gap)
    F proportional to 1/g^2 (at constant current)

This quadratic relationship means:
- Small current changes produce large force changes — fine control
  of preload is possible with a simple PWM driver.
- As the coupling seats (gap decreases), force increases rapidly —
  this is self-reinforcing preload, good for seating.
- A current limit sets a maximum crash force — the coupling will
  separate if the external force exceeds the magnetic preload,
  protecting the tooling.

### Dual-Use with Hall Effect Sensing

A ratiometric hall effect sensor (SS49E or A1302) placed in the
magnetic circuit serves two purposes simultaneously:

1. **Seating detection:** The hall sensor output changes sharply as
   the keeper approaches and the air gap closes. A threshold on the
   analog output gives binary seated/unseated detection.

2. **Z-axis force estimation:** Once seated, the hall sensor output
   varies with the magnetic flux, which varies with the preload
   current. By calibrating the sensor output against known loads,
   the hall sensor becomes a force sensor with no additional hardware.

This dual-use eliminates one sensor and its wiring from the BOM.

## 5. MEMS Flow Sensing Rationale

### Why Pressure Sensing Fails at Small Scale

Traditional vacuum part-presence detection uses a pressure switch or
analog pressure sensor on the vacuum line. This works well for large
suction cups (> 20mm) but fails for small-scale applications:

- **Leak rate vs. volume:** A small vacuum cup has a small trapped
  volume. Any leak (surface roughness, misalignment, porosity) is a
  large fraction of the total volume, making pressure unstable.
- **Response time:** Small volumes reach steady-state pressure quickly,
  but the pressure value depends on the balance between supply flow
  and leak flow — this balance is sensitive to supply pressure
  variation, line length, and fitting quality.
- **Binary failure mode:** Pressure sensing gives a threshold (part
  present/absent) but provides no diagnostic information about
  seal quality or degradation.

### Why MEMS Thermal Mass Flow is Superior

A MEMS thermal mass flow sensor (Omron D6F series) measures the actual
air flow rate in the vacuum line, not the pressure:

- **Flow is diagnostic:** Zero flow = perfect seal = part present.
  Small flow = partial seal = part present but warn. Large flow =
  no seal = part absent. This gives three states instead of two.
- **Insensitive to supply pressure:** Flow rate through a small leak
  is nearly independent of upstream pressure over a wide range
  (choked flow regime at the leak orifice).
- **Fast response:** MEMS thermal sensors respond in < 50ms.
- **Quantitative:** The analog output is proportional to flow rate,
  enabling threshold adjustment and trend monitoring (detect
  gradual seal degradation before failure).

The D6F-P0010A2 (0-10 L/min range) costs ~$28 and provides a
4.5V analog output directly compatible with any microcontroller ADC.
