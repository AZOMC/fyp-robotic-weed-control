# Architecture

## Split of responsibility

| | Raspberry Pi 4 | Arduino Mega 2560 |
|---|---|---|
| Runs | `pi/demo.py` | `firmware/Robot_Master` |
| Owns | vision, coordinates, policy, UI | kinematics, motion, safety limits |
| Language | Python | C++ |

The Pi decides *what* to pick. The Mega decides *how* to reach it and
refuses anything outside its workspace. Neither trusts the other blindly:
the Pi filters detections against the reach radius and bin keep-out before
sending, and the firmware validates every waypoint again before moving.

## Serial protocol

One line per command over Bluetooth at 9600 baud.

### Pi to Mega

| Command | Meaning |
|---|---|
| `PICK X<n> Y<n> [Z<n>]` | full pick-and-bin cycle; Z defaults to the standard depth |
| `F` `B` `L` `R` | drive forward / back / pivot left / right |
| `T` `X` | speed up / down |
| `0` | stop wheels **and** abort any arm move |
| `STATUS` | one-line machine-readable state dump |
| `JOGX<n>` `JOGY<n>` `JOGZ<n>` | one-shot relative Cartesian jog, mm |
| `SETHOME` `HOME` `WORK` `BIN` `PARK` | reference and canned positions |
| `E` `D` | enable everything / release everything |
| `GO` `GC` `GA<n>` | gripper open / close / angle |

`HELP` lists the full set.

### Mega to Pi

| Reply | Meaning |
|---|---|
| `DONE` | pick cycle succeeded, weed released over the bin |
| `ERR <reason>` | cycle failed — `TARGET`, `TRANSIT`, `BIN`, `BUSY`, `NOTHOMED` |
| `ST k=v k=v ...` | status line, parsed into the UI |

`DONE` is sent the instant the gripper opens over the bin, *before* the arm
travels home. The Pi may send the next detection immediately; bytes that
arrive during the return leg are parked by the firmware and replayed when
it finishes. That overlap is where a meaningful part of the cycle time
saving comes from.

## The autoweeding state machine

```
   SEARCH ──(weed in reach)──────────────────────────► PICK
     │                                                  │
     │ (nothing for 5 s)                                │ DONE / ERR
     ▼                                                  │
   ADVANCE ──(weed appears)──► STOP ──► SETTLE ──► REACQUIRE
     ▲                                                  │
     └──────────(lost it)◄──────────────────────────────┘
```

- **SEARCH** — poll the vision thread for an in-reach detection newer than
  the moment the state was entered.
- **ADVANCE** — drive forward, refreshing the command every 1.2 s. The
  firmware's own 5 s watchdog stops the wheels if the Pi goes quiet, so the
  refresh doubles as a deadman.
- **STOP / SETTLE / REACQUIRE** — stop from the Pi, wait for the chassis to
  settle, then require a detection from a frame captured *after* the stop.
  A coordinate measured while rolling is already stale.
- **PICK** — send, block for `DONE`/`ERR`, repeat.

## Concurrency

Four threads, with one lock that matters.

| Thread | Job |
|---|---|
| Vision | capture, inference, overlay, publish JPEG |
| Autoweeder | the state machine above |
| Housekeeping | Bluetooth reconnect, `STATUS` polling |
| Flask | HTTP requests, one MJPEG generator per viewer |

`SerialLink` holds a transmit lock for the whole request-reply exchange, so
a manual command physically cannot interleave with an in-flight pick. The
emergency stop deliberately bypasses that lock — a single `0` written
between two lines is exactly what the firmware's abort is designed to catch.

Vision publishes one annotated JPEG that every viewer reads, so additional
browser tabs cost nothing. An earlier version ran the whole pipeline inside
the Flask generator, which meant two tabs ran two inference loops fighting
over the same camera.

## Coordinate transform

A 4-point homography maps image pixels to robot millimetres:

```
[x_mm, y_mm, 1]^T  ~  M · [px, py, 1]^T
```

`M` is computed once by `cv2.getPerspectiveTransform` from four markers at
known positions and stored as `calibration_matrix.npy`. The inverse is used
to draw the reach circle and bin keep-out back onto the video overlay.

This assumes weeds lie on a plane at a known height. That holds for a fixed
camera over flat ground and is the main reason the system is bench-accurate
but not yet field-proven.

## Safety interlocks

1. Workspace cylinder — radius and Z limits, checked per interpolated point.
2. Bin keep-out — a volume the arm may never enter; checked at every point
   along a path, not just endpoints, so a move cannot clip the bin diagonally.
3. Drive watchdog — wheels stop 5 s after the last command.
4. `pickBusy` — a second `PICK` during a cycle is refused with `ERR BUSY`.
5. Mode arbitration on the Pi — see the README.
6. Emergency stop — aborts the arm move and stops the wheels, from any state.
