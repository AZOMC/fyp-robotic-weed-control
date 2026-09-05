# Hardware

## Bill of materials

Total build cost: approximately **RM 1,100 (~USD 250)**.

| Subsystem | Parts |
|---|---|
| Compute | Raspberry Pi 4 (4 GB), USB webcam, 5 V/3 A supply |
| Motion controller | Arduino Mega 2560, RAMPS 1.6 shield |
| Delta arm | 3x NEMA 17 steppers, 3x DRV8825 @ 1/32 microstepping, carbon rods, 3D-printed joints |
| Gripper | MG995 servo, 3D-printed claw |
| Drive | 2x L298N, 4x DC gear motors, 4x wheels |
| Comms | HC-05 Bluetooth module |
| Chassis | 2020 aluminium extrusion, acrylic panels, mesh bin |

## Pin map

Verified: no two subsystems share a pin.

### Delta steppers (RAMPS 1.6)

| Axis | STEP | DIR | EN |
|---|---|---|---|
| X | 54 | 55 | 38 |
| Y | 60 | 61 | 56 |
| Z | 46 | 48 | 62 |

Step pins are driven by direct `PORTF`/`PORTL` writes rather than
`digitalWrite`, which removes timing jitter from the pulse train.

### Drive motors (2x L298N)

| Side | ENA | ENB | IN1 | IN2 | IN3 | IN4 |
|---|---|---|---|---|---|---|
| Left | 2 | 3 | 25 | 23 | 17 | 16 |
| Right | 6 | 11 | 32 | 47 | 45 | 43 |

### Gripper

| Signal | Pin |
|---|---|
| Servo PWM | D4 |
| Servo V+ | **external 5–6 V supply** |
| Servo GND | common with Mega **and** the external supply |

### Bluetooth

HC-05 on `Serial1` — RX1 = D19, TX1 = D18, 9600 baud.

## Constraints learned the hard way

- **Never use D0/D1.** That is the bootloader's upload path.
- **Never power the MG995 from the RAMPS 5 V header.** Its stall current
  exceeds what the header can supply. External supply, common ground.
- **Timer conflicts.** `Servo.h` claims Timer5 on the Mega, which owns pins
  44/45/46. D46 is Z_STEP but is driven by direct port writes, and D45 is
  digital-only, so neither is affected. Nothing calls `analogWrite()` on
  44/45/46. Drive PWM uses Timer3 (D2/D3), Timer4 (D6) and Timer1 (D11).
- **D16/D17 are Serial2's TX2/RX2.** Serial2 is never started, so they are
  free as plain digital outputs.
- **Power quality is a first-order concern.** See the under-voltage finding
  in the main README — it was worth more than every software optimisation
  combined.

## Geometry

| Parameter | Value |
|---|---|
| Base triangle circumradius | 83.7 mm |
| End-effector circumradius | 30.4 mm |
| Upper arm length | 150 mm |
| Lower rod length | 280 mm |
| Gear ratio | 4.5 |
| Working radius | 120 mm (soft limit) |
| Z range | -130 mm (home) to -300 mm (pick depth) |
| Transit height | -180 mm (all XY travel happens here) |

The transit height sits above the bin keep-out plane, so no XY move can
clip the bin regardless of the weed coordinates. This was verified by
simulating every leg of the cycle for all legal weed positions on a 5 mm
grid — 1,710 positions, zero violations.
