# Calibration

The detector returns pixels. The arm needs millimetres. One 4-point
homography bridges them, and it has to be redone whenever the camera moves.

## Procedure

### 1. Place markers

Put at least four markers on the ground plane, inside the arm's reach, at
positions you can measure in millimetres from the arm's origin. Spread them
out — four points clustered in one corner give a poor fit. Two extra
markers are worth placing as validation points: measured, but deliberately
excluded from the fit, so the accuracy check is honest.

### 2. Capture

```bash
python calibrate.py capture
```

This waits for the exposure to settle before saving, rather than grabbing a
fixed number of warm-up frames — see [Exposure](#exposure) below. It writes
`calib_frame.jpg` and prints an `scp` command to pull it to your laptop.

### 3. Read pixel coordinates

Open the photo and read the pixel position of each marker centre (any image
editor's cursor readout will do). Put them in `calibration_points.json`:

```json
{
  "calibration_points": [
    {"name": "A", "pixel": [640,  95], "real_world": [0,  120]},
    {"name": "B", "pixel": [880, 300], "real_world": [100,  0]},
    {"name": "C", "pixel": [755, 620], "real_world": [0, -120]},
    {"name": "D", "pixel": [400, 340], "real_world": [-120, 0]}
  ],
  "validation_points": [
    {"name": "V1", "pixel": [570, 175], "real_world": [-40,  70]},
    {"name": "V2", "pixel": [595, 440], "real_world": [-50, -50]}
  ]
}
```

`real_world` is in millimetres, in the same sign convention the arm uses.

### 4. Compute and validate

```bash
python calibrate.py compute     # -> calibration_matrix.npy (backs up the old one)
python calibrate.py validate    # reports error in mm
python calibrate.py status      # what is currently on disk
```

`validate` reports error against both the fitted points and the held-out
validation points. The fitted points will always look good — the validation
points are the ones that tell you the truth.

## What good looks like

Error of a few millimetres at the validation points is fine: the gripper
has more tolerance than that. Tens of millimetres means a marker position
was mismeasured or a pixel coordinate was misread. If one point is much
worse than the rest, it is almost always that point, not the fit.

## Exposure

The USB camera used here exposes **no exposure controls at all** to V4L2 —
no `auto_exposure`, no `exposure_time_absolute`. Exposure is internal and
cannot be locked. Two consequences:

1. **Its auto-exposure settles slowly.** Measured on the machine, mean frame
   brightness went 239 → 219 → 193 → 168 → 153 and was still drifting after
   100 frames. A fixed warm-up count cannot be right, because the ramp is as
   long as the light demands. `camera_setup.settle()` waits for the
   brightness to actually stop changing.
2. **The factory defaults are wrong for this application** — `gain=128`,
   `brightness=128`, and `backlight_compensation` **on**, which deliberately
   over-exposes. `set_camera.sh` holds the tuned values, and
   `camera_setup.open_camera()` applies them on every open, including after
   a re-plug.

A calibration photo taken during the bright part of that ramp is unusable —
marker centres are hard to read and the homography inherits the error.

## When to recalibrate

- The camera has been knocked or remounted
- The camera height above the ground plane has changed
- The arm has been re-homed to a different physical rest position
- Validation error has crept up

`compute` automatically backs up the previous matrix with a timestamp, so
recalibrating is safe to try.
