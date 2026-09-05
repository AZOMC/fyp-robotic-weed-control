#!/usr/bin/env python3
"""
calibrate.py - one unified tool for the camera -> robot arm manual calibration.

Replaces compute_calibration.py + test_calibration.py + validate_calibration.py.
All the numbers you used to edit inside those three files now live in ONE
place: calibration_points.json. This script never needs to be edited.

USAGE (run on the Pi, inside the venv, from the fyp folder):

    python calibrate.py capture      # take a photo with the real camera, prints
                                      # the command to pull it to your laptop
    python calibrate.py compute      # read calibration_points.json -> save
                                      # calibration_matrix.npy (auto-backs up
                                      # the old one first)
    python calibrate.py validate     # check accuracy in mm using
                                      # validation_points (and/or calibration_points)
    python calibrate.py status       # show what's currently on disk

Full step-by-step recalibration procedure: see CALIBRATION_GUIDE.md.
"""

import argparse
import json
import os
import shutil
import socket
import sys
from datetime import datetime

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

import camera_setup

# ========= SETTINGS (kept in sync with production.py) =========
CAM_INDEX = 0
CAM_WIDTH = 1280
CAM_HEIGHT = 720
# WARMUP_FRAMES is gone: a fixed count cannot be right, because the ramp
# is as long as the light demands. camera_setup.settle() waits for the
# brightness to actually converge instead.

CALIBRATION_FILE = "calibration_matrix.npy"
POINTS_FILE = "calibration_points.json"
CAPTURE_FILE = "calib_frame.jpg"

# Where finished photos should land on the laptop. Adjust if this changes.
# Only used to print a copy-pasteable scp hint; nothing reads these.
LAPTOP_DEST_DIR = os.environ.get("LAPTOP_DEST_DIR", ".")
PI_USER = os.environ.get("PI_USER", "pi")
PI_PROJECT_DIR = os.environ.get("PI_PROJECT_DIR", "~/weedbot")
# =================================================================


def get_local_ip():
    """Best-effort LAN IP of the Pi, so the scp command can be copy-pasted as-is."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "<PI_IP>"
    finally:
        s.close()


def _frame_from_demo(url):
    """Pull one clean, un-annotated frame from a running demo.py."""
    import urllib.request
    endpoint = url.rstrip("/") + "/snapshot_raw.jpg"
    try:
        with urllib.request.urlopen(endpoint, timeout=10) as r:
            data = r.read()
            age = r.headers.get("X-Frame-Age", "?")
    except Exception as e:
        print("[INFO] demo.py not reachable at %s (%s)" % (endpoint, e))
        return None
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        print("[INFO] demo.py returned something that was not an image")
        return None
    print("[OK] Got a clean frame from demo.py (%ss old)" % age)
    return img


def cmd_capture(args):
    if cv2 is None:
        print("[ERROR] opencv (cv2) not available in this environment.")
        sys.exit(1)

    # open_camera applies set_camera.sh; settle waits for the picture to
    # stop changing instead of counting frames. Measured on this camera:
    # at factory defaults a 10-frame warmup captured mean brightness 220
    # and a 50-frame warmup still captured 184 (255 = pure white), because
    # the internal auto-exposure ramps for well over 100 frames and there
    # is no exposure control to lock. See camera_setup.py.
    frame = None
    if not args.from_demo:
        cap = camera_setup.wait_for_camera(CAM_INDEX, CAM_WIDTH, CAM_HEIGHT,
                                           timeout=args.wait)
        if cap is not None:
            print("[INFO] Waiting for the exposure to settle...")
            frame, info = camera_setup.settle(cap)
            cap.release()
            if frame is not None and getattr(frame, "size", 0):
                if not info["settled"]:
                    print("[WARN] The image had not stabilised - calibrate "
                          "from this photo only if it looks correctly exposed.")
                print("[INFO] mean brightness %.0f" % info["mean"])
            else:
                frame = None

    if frame is None:
        # Fall back to demo.py. Direct access to this camera is unreliable:
        # the node wedges, and every cv2 open on a wedged node blocks for
        # ~50 s. demo.py retries in the background and does eventually get
        # through, and its /snapshot_raw.jpg is a clean un-annotated frame
        # from the same camera at the same resolution - exactly what
        # calibration needs. Using it beats fighting for the device.
        print("[INFO] Trying a running demo.py at %s ..." % args.url)
        frame = _frame_from_demo(args.url)

    if frame is None:
        print("[ERROR] Could not get a frame either way.")
        print("        The camera enumerates but does not respond - that is")
        print("        the USB fault, not a software problem (lsusb, dmesg).")
        print()
        print("        Way through it: start demo.py in another terminal,")
        print("        wait for '[APP] camera 0 open at 1280x720', then run")
        print("            python calibrate.py capture --from-demo")
        sys.exit(1)

    cv2.imwrite(CAPTURE_FILE, frame)
    print("[OK] Saved %s (%dx%d)" % (CAPTURE_FILE, frame.shape[1], frame.shape[0]))

    ip = get_local_ip()
    scp_cmd = f'scp {PI_USER}@{ip}:{PI_PROJECT_DIR}/{CAPTURE_FILE} "{LAPTOP_DEST_DIR}/{CAPTURE_FILE}"'

    print()
    print("=" * 70)
    print("NEXT STEP - run this on your LAPTOP (PowerShell / cmd), not the Pi:")
    print()
    print(f"    {scp_cmd}")
    print()
    print("Then open the image on your laptop and read off pixel coordinates")
    print("for each marker (Paint's status bar, or run pick_pixels.py - see")
    print("CALIBRATION_GUIDE.md), measure each marker's real-world position")
    print(f"in mm from the arm's origin, and put both into {POINTS_FILE}.")
    print("=" * 70)


def load_points():
    if not os.path.exists(POINTS_FILE):
        print(f"[ERROR] {POINTS_FILE} not found. See CALIBRATION_GUIDE.md.")
        sys.exit(1)
    with open(POINTS_FILE) as f:
        data = json.load(f)
    return data


def _to_arrays(points):
    names = [p["name"] for p in points]
    px = np.array([p["pixel"] for p in points], dtype=np.float32)
    rw = np.array([p["real_world"] for p in points], dtype=np.float32)
    return names, px, rw


def cmd_compute(args):
    data = load_points()
    points = data.get("calibration_points", [])

    if len(points) < 4:
        print(f"[ERROR] Need at least 4 calibration_points, found {len(points)}.")
        sys.exit(1)

    names, px, rw = _to_arrays(points)
    print(f"[INFO] Using {len(points)} calibration points: {', '.join(names)}")

    if len(points) == 4:
        M = cv2.getPerspectiveTransform(px, rw)
        method = "exact 4-point perspective transform"
    else:
        M, mask = cv2.findHomography(px, rw, method=cv2.RANSAC, ransacReprojThreshold=3.0)
        method = f"homography fit (RANSAC) over {len(points)} points"
        if mask is not None:
            inliers = [n for n, m in zip(names, mask.ravel()) if m]
            outliers = [n for n, m in zip(names, mask.ravel()) if not m]
            print(f"[INFO] Inliers: {inliers}")
            if outliers:
                print(f"[WARN] Flagged as outliers (double-check these measurements): {outliers}")

    if M is None:
        print("[ERROR] Could not compute a transform from these points (check for duplicate/collinear points).")
        sys.exit(1)

    if os.path.exists(CALIBRATION_FILE):
        backup = f"calibration_matrix_backup_{datetime.now():%Y%m%d_%H%M%S}.npy"
        shutil.copy2(CALIBRATION_FILE, backup)
        print(f"[INFO] Backed up old matrix -> {backup}")

    np.save(CALIBRATION_FILE, M)
    print(f"[OK] Saved {CALIBRATION_FILE} using {method}")
    print("Matrix:")
    print(M)
    print()
    print("Run 'python calibrate.py validate' next.")
    print("If production.py is currently running, restart it to pick up the new matrix.")


def _pixel_to_realworld(M, px, py):
    point = np.array([[[px, py]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(point, M)
    return transformed[0][0]


def cmd_validate(args):
    if not os.path.exists(CALIBRATION_FILE):
        print(f"[ERROR] {CALIBRATION_FILE} not found. Run 'python calibrate.py compute' first.")
        sys.exit(1)

    M = np.load(CALIBRATION_FILE)
    data = load_points()

    def run_set(points, label):
        if not points:
            return
        print(f"\n--- {label} ---")
        errors = []
        for p in points:
            px, py = p["pixel"]
            ex, ey = p["real_world"]
            real = _pixel_to_realworld(M, px, py)
            err = float(np.hypot(real[0] - ex, real[1] - ey))
            errors.append(err)
            print(f"{p['name']:>4}: pixel=({px},{py}) -> predicted=({real[0]:.1f},{real[1]:.1f}) "
                  f"expected=({ex},{ey})  error={err:.1f} mm")
        print(f"Mean error: {np.mean(errors):.1f} mm | Max error: {np.max(errors):.1f} mm")

    validation_points = data.get("validation_points", [])
    calibration_points = data.get("calibration_points", [])

    if validation_points:
        run_set(validation_points, "Validation points (independent - the real accuracy check)")
    else:
        print("[WARN] No validation_points in calibration_points.json.")
        print("       Testing against calibration_points instead - note this only checks")
        print("       self-consistency (with exactly 4 points the error will be ~0 by")
        print("       construction, that does NOT prove the calibration is accurate).")
        print("       Add at least one extra measured marker to validation_points for a")
        print("       real accuracy check. See CALIBRATION_GUIDE.md.")
        run_set(calibration_points, "Calibration points (self-consistency only)")


def cmd_status(args):
    print(f"Working directory: {os.getcwd()}\n")

    if os.path.exists(CALIBRATION_FILE):
        mtime = datetime.fromtimestamp(os.path.getmtime(CALIBRATION_FILE))
        M = np.load(CALIBRATION_FILE)
        print(f"{CALIBRATION_FILE}: last updated {mtime:%Y-%m-%d %H:%M:%S}")
        print(M)
    else:
        print(f"{CALIBRATION_FILE}: NOT FOUND")

    print()
    if os.path.exists(POINTS_FILE):
        data = load_points()
        cal = data.get("calibration_points", [])
        val = data.get("validation_points", [])
        print(f"{POINTS_FILE}: {len(cal)} calibration point(s), {len(val)} validation point(s)")
        for p in cal:
            print(f"  [calib] {p['name']}: pixel={p['pixel']} real_world={p['real_world']}")
        for p in val:
            print(f"  [valid] {p['name']}: pixel={p['pixel']} real_world={p['real_world']}")
    else:
        print(f"{POINTS_FILE}: NOT FOUND")


def main():
    parser = argparse.ArgumentParser(description="Unified manual camera->robot calibration tool.")
    sub = parser.add_subparsers(dest="command", required=True)

    cap_p = sub.add_parser(
        "capture",
        help="Take a photo with the camera and print the scp command to fetch it.")
    cap_p.add_argument("--wait", type=float, default=90.0,
                       help="seconds to keep retrying the camera (it "
                            "re-enumerates; default 90)")
    cap_p.add_argument("--from-demo", action="store_true",
                       help="skip the camera and take the frame straight from "
                            "a running demo.py")
    cap_p.add_argument("--url", default="http://127.0.0.1:5000",
                       help="where demo.py is listening")
    sub.add_parser("compute", help="Compute calibration_matrix.npy from calibration_points.json.")
    sub.add_parser("validate", help="Check accuracy in mm using validation_points.")
    sub.add_parser("status", help="Show current matrix and points on disk.")

    args = parser.parse_args()
    {
        "capture": cmd_capture,
        "compute": cmd_compute,
        "validate": cmd_validate,
        "status": cmd_status,
    }[args.command](args)


if __name__ == "__main__":
    main()
