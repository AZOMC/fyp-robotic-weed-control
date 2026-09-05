#!/usr/bin/env python3
"""
camera_setup.py - one place that knows how to bring this camera up sanely.

WHY THIS EXISTS
---------------
The USB camera on this robot (349c:3307 "Generic HD video") exposes NO
exposure controls to V4L2 at all - checked with v4l2-ctl --list-ctrls-menus,
there is no auto_exposure and no exposure_time_absolute. Exposure is
entirely internal and cannot be locked. Everything you can actually set
is post-processing: brightness, contrast, gain, white balance, backlight
compensation.

Two consequences, both of which have bitten this project:

1. At factory defaults the picture is much brighter than the tuned look,
   because the defaults are gain=128 (tuned: 70), brightness=128 (110),
   and backlight_compensation=128 i.e. ON (0) - backlight compensation
   deliberately over-exposes. white_balance_automatic also defaults to
   ON, so the colour hunts continuously. set_camera.sh fixes all of
   that, but NOTHING used to run it: production.py, stream_ncnn.py and
   calibrate.py all just opened the device, so unless it had been run by
   hand that session, every capture used the bright defaults.

2. The internal auto-exposure settles SLOWLY. Measured on the machine,
   mean frame brightness went 239 -> 219 -> 193 -> 168 -> 153 and was
   still drifting after 100 frames. calibrate.py's WARMUP_FRAMES = 10
   captured deep inside the blown-out part of that ramp. A fixed frame
   count cannot be right, because the ramp length depends on the light;
   settle() below waits for the picture to actually stop changing.

Import this from calibrate.py, stream_ncnn.py and demo.py rather than
copying the logic around.
"""

import os
import subprocess
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

# Mirrors set_camera.sh. A LIST, not a dict, because order matters:
# white_balance_temperature reads back "flags=inactive" and is ignored
# while white_balance_automatic is still on, so auto must go off first.
DEFAULT_CONTROLS = [
    ("contrast", 140),
    ("saturation", 90),
    ("sharpness", 100),
    ("backlight_compensation", 0),
    ("white_balance_automatic", 0),
    ("white_balance_temperature", 140),
    ("brightness", 110),
    ("gain", 70),
]

SCRIPT_NAME = "set_camera.sh"


def _here(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)


def apply_controls(dev="/dev/video0", script=None, log=print):
    """Push the tuned v4l2 controls at the camera.

    Prefers set_camera.sh so there is a single file to tune, and falls
    back to DEFAULT_CONTROLS if it is missing. Safe to call repeatedly,
    and safe to call while streaming - the controls take effect live.
    Returns True if anything was applied.
    """
    if os.name != "posix" or not os.path.exists(dev):
        return False

    script = script or _here(SCRIPT_NAME)
    if os.path.exists(script):
        try:
            r = subprocess.run(["bash", script], capture_output=True,
                               text=True, timeout=20)
            if r.returncode == 0:
                log("[cam] applied %s" % os.path.basename(script))
                return True
            log("[cam] %s failed: %s" % (os.path.basename(script),
                                         (r.stderr or r.stdout).strip()[:200]))
        except Exception as e:
            log("[cam] could not run %s: %s" % (script, e))

    ok = 0
    for name, value in DEFAULT_CONTROLS:
        try:
            r = subprocess.run(
                ["v4l2-ctl", "-d", dev, "--set-ctrl=%s=%d" % (name, value)],
                capture_output=True, text=True, timeout=10)
            ok += (r.returncode == 0)
        except Exception:
            pass
    if ok:
        log("[cam] applied %d built-in controls" % ok)
    return ok > 0


def open_camera(index=0, width=1280, height=720, mjpg=True, apply=True,
                log=print):
    """Open the camera the way every script in this project should.

    Uses the V4L2 backend explicitly - left to choose, OpenCV falls
    through to FFMPEG and prints a misleading libavdevice warning when
    the real problem is that the device is simply not there.
    """
    if cv2 is None:
        raise RuntimeError("opencv not available")

    if hasattr(cv2, "CAP_V4L2") and os.name == "posix":
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(index)
    if mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    if not cap.isOpened():
        return None

    # Controls are applied AFTER the open. They survive it either way
    # (verified on the machine), but applying afterwards also covers a
    # camera that was re-plugged and came back at its defaults.
    if apply:
        apply_controls("/dev/video%d" % index, log=log)
    return cap


def node_healthy(dev="/dev/video0", timeout=2.0):
    """Cheap liveness check on a video node, WITHOUT going through OpenCV.

    This matters more than it sounds. When the camera has enumerated but
    is not actually responding, the node exists and cv2.VideoCapture()
    blocks on it for ~50 seconds before failing. A retry loop built on
    VideoCapture alone therefore manages about two attempts a minute,
    which is far too few to catch one of the brief windows where the
    camera is healthy. v4l2-ctl --info fails in under a second on a
    wedged node, so this gates the expensive call and turns those two
    attempts a minute into roughly sixty.
    """
    if not os.path.exists(dev):
        return False
    try:
        r = subprocess.run(["v4l2-ctl", "-d", dev, "--info"],
                           capture_output=True, text=True, timeout=timeout)
    except Exception:
        return False
    return r.returncode == 0 and "Card type" in (r.stdout or "")


def wait_for_camera(index=0, width=1280, height=720, timeout=60.0, log=print):
    """open_camera(), but keep trying while the camera re-enumerates.

    This camera drops off the USB bus and comes back with a new device
    number, so a single open() attempt is a coin flip: it fails if you
    happen to call it during a re-enumeration. Retrying across that
    window is the difference between "capture failed" and "capture
    worked, it just took 12 seconds".

    This is a workaround for a physical fault, not a fix for it - the
    enumeration errors are electrical (see the module docstring).
    """
    dev = "/dev/video%d" % index
    t0 = time.time()
    attempt = 0
    probes = 0
    said = False
    while time.time() - t0 < timeout:
        # Fast gate first - see node_healthy(). Only spend a slow
        # VideoCapture open on a node that is actually answering.
        if not node_healthy(dev):
            probes += 1
            if not said:
                log("[cam] %s is not responding - retrying for up to %.0fs "
                    "(the camera re-enumerates; this is the USB fault)"
                    % (dev, timeout))
                said = True
            time.sleep(1.0)
            continue

        attempt += 1
        cap = open_camera(index, width, height, log=(log if attempt == 1
                                                     else lambda m: None))
        if cap is not None:
            # An open that succeeds mid-disconnect still fails on read,
            # so prove it delivers a frame before handing it back.
            ok, frame = cap.read()
            if ok and frame is not None:
                if attempt > 1 or probes:
                    log("[cam] camera came back after %.0fs "
                        "(%d probes, %d opens)"
                        % (time.time() - t0, probes, attempt))
                return cap
            cap.release()
        time.sleep(1.0)
    log("[cam] gave up after %.0fs (%d probes, %d opens) - the node never "
        "answered. The camera is enumerating but not working; this is the "
        "USB fault, not a software problem."
        % (time.time() - t0, probes, attempt))
    return None


def settle(cap, timeout=12.0, tol=2.0, window=6, min_frames=8, log=print):
    """Discard frames until the picture stops changing, then return one.

    Replaces "read N frames and hope". N cannot be right in general: the
    auto-exposure ramp is as long as the light demands, and on this
    camera it has been measured at well over 100 frames. Instead this
    watches the mean frame brightness and stops when the last `window`
    means span less than `tol`, i.e. the image has actually converged.

    Returns (frame, info). info has the frame count, elapsed time, final
    mean, and settled=False if it ran out of time - a caller that cares
    can warn rather than silently using a half-exposed frame.
    """
    if np is None:
        raise RuntimeError("numpy not available")

    means = []
    frames = 0
    first = None
    frame = None
    misses = 0
    t0 = time.time()

    while time.time() - t0 < timeout:
        ok, f = cap.read()
        if not ok or f is None:
            # Do NOT spin silently here. A camera that has fallen off the
            # USB bus fails every read, and without this the loop burned
            # the whole timeout and then reported a meaningless
            # "brightness 0 -> 0". Fail fast and say what is wrong.
            misses += 1
            if misses >= 40:
                log("[cam] ERROR: %d consecutive read failures - the camera "
                    "stopped delivering frames. Check 'lsusb' and "
                    "'dmesg | tail': this camera has a history of dropping "
                    "off the USB bus." % misses)
                return frame, {"frames": frames, "seconds": time.time() - t0,
                               "mean": means[-1] if means else None,
                               "first_mean": first, "settled": False,
                               "read_failed": True}
            time.sleep(0.05)
            continue
        misses = 0
        frame = f
        frames += 1
        m = float(np.mean(f))
        if first is None:
            first = m
        means.append(m)
        if len(means) > window:
            means.pop(0)
        if frames >= min_frames and len(means) == window:
            if max(means) - min(means) < tol:
                el = time.time() - t0
                log("[cam] settled after %d frames / %.1fs "
                    "(brightness %.0f -> %.0f)" % (frames, el, first, m))
                return frame, {"frames": frames, "seconds": el, "mean": m,
                               "first_mean": first, "settled": True}

    el = time.time() - t0
    if not means:
        log("[cam] ERROR: no frames at all in %.1fs - camera present but not "
            "streaming." % el)
        return frame, {"frames": 0, "seconds": el, "mean": None,
                       "first_mean": None, "settled": False,
                       "read_failed": True}
    m = means[-1]
    log("[cam] WARNING: still drifting after %d frames / %.1fs "
        "(brightness %.0f -> %.0f) - the image may be over-exposed. "
        "Check the light, or raise the timeout."
        % (frames, el, first, m))
    return frame, {"frames": frames, "seconds": el, "mean": m,
                   "first_mean": first, "settled": False,
                   "read_failed": False}


if __name__ == "__main__":
    # Quick manual check:  ./venv/bin/python camera_setup.py
    import sys
    cap = wait_for_camera()
    if cap is None:
        sys.exit("could not open the camera - is /dev/video0 there? "
                 "(check 'lsusb -t' and 'dmesg | tail')")
    frame, info = settle(cap)
    cap.release()
    print("result:", info)
    if frame is not None and getattr(frame, "size", 0):
        cv2.imwrite("camera_setup_test.jpg", frame)
        print("wrote camera_setup_test.jpg")
