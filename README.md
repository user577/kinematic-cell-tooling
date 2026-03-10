# Kinematic Cell Tooling Platform

A family of precision robot cell tooling products built on exact-constraint
kinematic coupling principles. All products share a common design language:
standard purchased hardware (ground dowel pins, precision ball bearings,
neodymium magnets, hall effect sensors) assembled into machined aluminum
bodies. No exotic manufacturing required.

## Products in this repository

### 1. Intelligent Tooling Interface (ITI)
Sits between robot flange and end-of-arm tooling. Provides:
- Exact-constraint kinematic coupling (Kelvin coupling, 3-ball/2-pin-cradle)
- Variable electromagnetic preload (programmable compliance and crash force)
- Hall effect seating detection and Z-axis force sensing
- Inline vacuum flow sensing for part presence (MEMS thermal mass flow)
- Tooling-face inductive part presence sensing

### 2. Smart Heat Set End Effector
Robot-mounted heat set insert installation tool. Provides:
- Resistive heated tip with K-type thermocouple
- PEEK thermal break collar as Z surface reference
- Strain gauge flexure for two-event force signature sensing
- State machine: DESCENDING -> SURFACE_FOUND -> PRESSING ->
  SOFTENING_DIP -> SEATING -> BOTTOMED -> DWELL -> COMPLETE
- Compatible with heat set insert feeder

### 3. Kinematic Workholding Pallet System
Precision pallet location and retention for robot assembly cells. Provides:
- Kelvin coupling base station (sub-0.05mm XY+theta repeatability)
- Cam-driven twist-lock rotary latch (axial retention, no tipping)
- Optional solenoid release (24VDC, standard robot IO)
- Hall effect seating confirmation
- Light pallets (latch handles retention, not mass)

### 4. Kinematic Vision Mount
Exact-repeat camera positioning for robot cells. Allows camera removal
and replacement without recalibration.

## Design Philosophy
Every product in this family exploits a physical principle the commercial
incumbents (ATI, Schunk, Zimmer, Kistler) never adopted because they
optimized a different mechanism. The goal is to undercut commercial pricing
by 5-10x while meeting or exceeding performance on the metrics that matter
for robot cell applications.

## Manufacturing Constraints
- All machining: manual mill, lathe, drill press
- Target tolerances: +/-0.025mm on critical features
- All sensors and hardware: available from Digi-Key, Mouser, McMaster-Carr
- No custom ICs, no PCB fab beyond protoboard signal conditioning

## License
CERN Open Hardware Licence Version 2 - Permissive
