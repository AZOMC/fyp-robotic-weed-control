#!/usr/bin/env python3
"""
demo.py - single Pi-hosted app: live detection stream + full web control panel.

Supersedes production.py's "run it and it just picks" behaviour with an
explicit, mode-arbitrated system that you drive from a browser:

  * live MJPEG stream with FPS / inference-time / detection overlays
  * autoweeding toggle (detect -> PICK -> wait DONE -> repeat)
  * platform auto-advance: no weed for N seconds -> drive forward until a
    weed comes into reach -> stop from the PI (accurate coords, no creep)
    -> settle -> re-acquire a FRESH detection -> pick
  * manual Mega control: enable/release, SETHOME, Cartesian jog, gripper,
    canned positions, raw command console
  * manual platform driving (F/B/L/R, speed up/down, stop)
  * mode switching with no restart, and hard arbitration so a manual
    command can never land in the middle of a pick cycle

RUN ON THE PI
-------------
    cd ~/fyp
    source ~/fypenv/bin/activate      # or however fypenv is activated
    sudo -v                           # cache sudo creds for the rfcomm bind
    python demo.py

It binds /dev/rfcomm0 to the HC-05 itself on startup (see BT_MAC below), so
you no longer have to run the rfcomm bind command by hand. See --help for
--no-bind, --bt-mac, --model, --cam and friends.

Then open   http://<PI_IP>:5000   from the laptop on the same wifi.

FIRMWARE
--------
Needs the updated Robot_Master.ino, which adds two commands this app uses:
    STATUS            -> one parsable "ST k=v k=v ..." line
    JOGX/JOGY/JOGZ<n> -> one-shot relative Cartesian jog in mm
Everything else it sends is stock firmware (PICK, F/B/L/R/0, E/D, SETHOME).
"""

import argparse
import collections
import os
import shutil
import socket
import subprocess
import tempfile
import threading
import time

# OpenCV prints V4L2 probe failures straight from C++ on every camera
# retry. Set before importing cv2 or it has no effect.
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request

try:
    import serial
except ImportError:
    serial = None

# Shared camera bring-up (tuned v4l2 controls + adaptive exposure settle).
# Optional so a demo.py copied on its own still runs - it just loses the
# automatic control application.
try:
    import camera_setup
except ImportError:
    camera_setup = None


# ======================================================================
#  DEFAULTS  (all overridable from the command line)
# ======================================================================
MODEL_PATH   = "best_v2_test_ncnn_model"
CAM_INDEX    = 0
IMG_SIZE     = 320
CONF_THRESH  = 0.30
CAM_WIDTH    = 1280
CAM_HEIGHT   = 720
JPEG_QUALITY = 60

BT_PORT   = "/dev/rfcomm0"
BT_MAC    = os.environ.get("BT_MAC", "00:00:00:00:00:00")  # your HC-05 - see README
BT_CHAN   = 1
BAUD_RATE = 9600

CALIBRATION_FILE = "calibration_matrix.npy"
HTTP_PORT = 5000

# Mirror of the firmware's SAFE_R: detections further out than this are
# drawn and reported but never picked. Keep it <= the firmware's softRmax.
SAFE_R_MM = 115.0

# Mirrors the firmware's keepX. Overwritten by the kpx value the Mega
# reports in STATUS as soon as it connects.
KEEP_X_MM = 60.0


# ======================================================================
#  EVENT LOG - one ring buffer for app events, TX lines and Mega replies.
#  Doubles as the reply bus: SerialLink waits on it for DONE/ERR.
# ======================================================================
class EventLog:
    def __init__(self, maxlen=800):
        self._d = collections.deque(maxlen=maxlen)
        self._seq = 0
        self._cv = threading.Condition()

    def add(self, src, text):
        text = str(text).rstrip()
        if not text:
            return None
        with self._cv:
            self._seq += 1
            entry = (self._seq, time.time(), src, text)
            self._d.append(entry)
            self._cv.notify_all()
        print("[%s] %s" % (src.upper(), text), flush=True)
        return self._seq

    def seq(self):
        with self._cv:
            return self._seq

    def since(self, seq):
        with self._cv:
            rows = [e for e in self._d if e[0] > seq]
            return rows, self._seq

    def wait(self, pred, timeout, since_seq):
        """Block until a logged entry satisfies pred(src, text). Returns the
        text of the matching entry, or None on timeout."""
        deadline = time.time() + timeout
        cursor = since_seq
        with self._cv:
            while True:
                for s, _ts, src, text in list(self._d):
                    if s <= cursor:
                        continue
                    cursor = s
                    if pred(src, text):
                        return text
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._cv.wait(min(remaining, 0.25))


LOG = EventLog()


# ----------------------------------------------------------------------
#  The browser polls /api/status about twice a second and /api/log more
#  often than that. Werkzeug logs every one of them, which buries the
#  robot's own messages under hundreds of lines of HTTP noise. Keep the
#  interesting requests (page loads, stream, control POSTs) and drop the
#  polling.
# ----------------------------------------------------------------------
def _quieten_werkzeug():
    import logging

    class DropPolling(logging.Filter):
        NOISE = ("/api/status", "/api/log", "/favicon.ico")

        def filter(self, record):
            msg = record.getMessage()
            return not any(n in msg for n in self.NOISE)

    # Filter only - do NOT raise the level, or the requests worth seeing
    # (page loads, /stream, every control POST) disappear too.
    logging.getLogger("werkzeug").addFilter(DropPolling())


# ======================================================================
#  RUNTIME SETTINGS  (tunable live from the web UI)
# ======================================================================
class Settings:
    def __init__(self):
        self.lock = threading.Lock()
        self.conf_thresh = CONF_THRESH
        self.safe_r      = SAFE_R_MM
        # Bin keep-out. A PICK descends to pickZ (-300), which is below
        # the firmware's keepZ, so the descent column is inside the bin
        # volume for ANY weed at X >= keepX - the Mega answers ERR TARGET.
        # Filtering here means the auto loop never wastes a cycle on one.
        # Kept in sync from the Mega's STATUS (kpx), so the firmware stays
        # the single source of truth.
        self.keep_x      = KEEP_X_MM
        # auto-weeding behaviour
        self.search_timeout  = 5.0    # s with no in-reach weed -> drive forward
        self.settle_s        = 0.45   # s to let the platform stop after 0
        self.reacquire_s     = 1.8    # s to re-find the weed after stopping
        self.drive_refresh_s = 1.2    # re-send F (Mega watchdog is 5 s)
        self.advance_max_s   = 25.0   # give up driving after this, take a breath
        self.post_pick_s     = 0.35   # settle after DONE before looking again
        self.pick_timeout_s  = 25.0   # wait for DONE/ERR
        self.det_max_age     = 0.60   # a detection older than this is stale
        # overlay
        self.show_boxes = True
        self.show_reach = True
        # manual
        self.jog_step = 10.0

    def snapshot(self):
        with self.lock:
            return dict((k, v) for k, v in self.__dict__.items() if k != "lock")

    def update(self, data):
        applied = {}
        with self.lock:
            for k, v in data.items():
                if k == "lock" or not hasattr(self, k):
                    continue
                cur = getattr(self, k)
                try:
                    val = bool(v) if isinstance(cur, bool) else type(cur)(v)
                except (TypeError, ValueError):
                    continue
                setattr(self, k, val)
                applied[k] = val
        return applied


SET = Settings()


# ======================================================================
#  RFCOMM  -  getting /dev/rfcomm0 to actually be connected
# ----------------------------------------------------------------------
#  This used to run "rfcomm bind", which is what you typed by hand. Bind
#  is documented to raise the connection when the node is opened, and on
#  this Pi it does not: measured on the robot, the node sits in state
#  "clean", open() succeeds, every read returns zero bytes, and the state
#  flips to "closed". That is the whole reason the app could claim a link
#  that did not exist - the tty is real either way.
#
#  "rfcomm connect" raises the link immediately and holds it for as long
#  as the process lives:
#
#     rfcomm0: D8:3A:.. -> AA:BB:CC:DD:EE:FF channel 1 connected
#              [reuse-dlc release-on-hup tty-attached]
#
#  So the connection is now a child process this app owns and supervises.
#  If the robot is powered off the child exits, which is a genuine signal
#  that the link is down rather than something to be guessed at.
# ======================================================================
class Rfcomm:
    def __init__(self, dev, mac, chan, port_path, enabled=True):
        self.dev = dev
        self.mac = mac
        self.chan = chan
        self.port_path = port_path
        self.enabled = enabled
        self.proc = None
        self.state = "idle"        # idle|connecting|connected|failed|disabled
        self.last_error = ""
        self._out = None
        self._lock = threading.Lock()
        if not enabled:
            self.state = "disabled"

    # -------- helpers --------
    def _sudo(self, *args):
        base = [] if (os.name == "posix" and os.geteuid() == 0) else ["sudo", "-n"]
        return base + ["rfcomm"] + list(args)

    def link_connected(self):
        """Authoritative: ask rfcomm itself, do not infer from our own state."""
        if os.name != "posix" or shutil.which("rfcomm") is None:
            return False
        try:
            r = subprocess.run(["rfcomm", "-a"], capture_output=True,
                               text=True, timeout=8)
        except Exception:
            return False
        for line in (r.stdout or "").splitlines():
            if line.strip().startswith("rfcomm%d:" % self.dev):
                return " connected" in line
        return False

    def link_line(self):
        try:
            r = subprocess.run(["rfcomm", "-a"], capture_output=True,
                               text=True, timeout=8)
        except Exception:
            return ""
        for line in (r.stdout or "").splitlines():
            if line.strip().startswith("rfcomm%d:" % self.dev):
                return line.strip()
        return ""

    def alive(self):
        return (self.proc is not None and self.proc.poll() is None
                and self.link_connected())

    # -------- lifecycle --------
    def start(self, timeout=25.0):
        if not self.enabled:
            return False
        with self._lock:
            if self.alive():
                self.state = "connected"
                return True
            self._kill()

            if os.name != "posix":
                self.state = "disabled"
                return False
            if shutil.which("rfcomm") is None:
                self._fail("rfcomm not installed - sudo apt install bluez")
                return False

            self.state = "connecting"
            LOG.add("app", "rfcomm connect %d %s ch%d ..."
                    % (self.dev, self.mac, self.chan))

            # Two attempts. A DLC left over from a previous run (or from a
            # process that was killed rather than asked to stop) makes the
            # first connect fail with "Device or resource busy"; the
            # release below clears it, but the kernel needs a moment to
            # actually tear it down, so the retry is what succeeds.
            for attempt in (1, 2):
                subprocess.run(self._sudo("release", str(self.dev)),
                               capture_output=True, text=True, timeout=10)
                if attempt == 2:
                    time.sleep(2.0)
                try:
                    self._out = tempfile.TemporaryFile(mode="w+")
                    self.proc = subprocess.Popen(
                        self._sudo("connect", str(self.dev), self.mac,
                                   str(self.chan)),
                        stdout=self._out, stderr=subprocess.STDOUT)
                except Exception as e:
                    self._fail("could not start rfcomm connect: %s" % e)
                    return False

                deadline = time.time() + timeout
                died = None
                while time.time() < deadline:
                    if self.proc.poll() is not None:
                        died = self._why_it_died()
                        break
                    if self.link_connected():
                        self.state = "connected"
                        self.last_error = ""
                        LOG.add("app", "Bluetooth link up: %s" % self.link_line())
                        return True
                    time.sleep(0.4)

                retryable = died and ("busy" in died.lower()
                                      or "in use" in died.lower())
                if attempt == 1 and retryable:
                    LOG.add("app", "Bluetooth: %s - releasing and retrying" % died)
                    self._kill()
                    continue
                self._fail(died or
                           ("no Bluetooth link to %s after %.0fs - is the "
                            "HC-05 powered and in range?" % (self.mac, timeout)))
                return False
            return False

    def _why_it_died(self):
        """rfcomm connect prints the real reason before exiting - use it."""
        text = ""
        try:
            self._out.seek(0)
            text = self._out.read().strip().replace("\n", "; ")
        except Exception:
            pass
        if "not permitted" in text.lower() or "password" in text.lower():
            return ("rfcomm connect needs root: run 'sudo -v' before demo.py, "
                    "or add a NOPASSWD sudoers rule for /usr/bin/rfcomm")
        return text or "rfcomm connect exited immediately"

    def _fail(self, msg):
        self.state = "failed"
        self.last_error = msg
        self._kill()
        LOG.add("app", "Bluetooth: %s" % msg)

    def _kill(self):
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=5)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None
        if self._out is not None:
            try:
                self._out.close()
            except Exception:
                pass
            self._out = None

    def stop(self):
        with self._lock:
            self._kill()
            if os.name == "posix" and shutil.which("rfcomm"):
                try:
                    subprocess.run(self._sudo("release", str(self.dev)),
                                   capture_output=True, text=True, timeout=10)
                except Exception:
                    pass
            if self.state != "disabled":
                self.state = "idle"

    def status(self):
        return {"state": self.state, "connected": self.link_connected(),
                "detail": self.link_line(), "error": self.last_error,
                "enabled": self.enabled}


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "0.0.0.0"
    finally:
        s.close()


# ======================================================================
#  SERIAL LINK
# ----------------------------------------------------------------------
#  ONE reader thread owns ser.read(). Everything else talks through
#  command() / write_line(), both of which take _tx, so two writers can
#  never interleave bytes mid-line. command() holds _tx for the WHOLE
#  request->reply exchange, which is what stops a manual raw command from
#  landing inside an in-flight PICK.
#
#  interrupt() deliberately bypasses _tx: an emergency stop must go out
#  even while a 25-second pick cycle owns the lock. A bare "0" written
#  between two full lines is exactly what the firmware's
#  0-at-start-of-line abort is designed to receive.
# ======================================================================
HANDSHAKE_TIMEOUT = 6.0     # seconds to wait for the Mega to answer STATUS


class SerialLink:
    def __init__(self, port, baud, rfcomm=None):
        self.port = port
        self.baud = baud
        # Optional Rfcomm manager. Having it lets the failure message name
        # the half that actually broke: no radio link at all, versus a
        # perfectly good radio link with a silent Mega behind it. Those
        # two need completely different fixes, so they must not share a
        # single vague "not connected".
        self.rfcomm = rfcomm
        self._ser = None
        self._tx = threading.Lock()
        self._stop = threading.Event()
        self._reader = None
        # disconnected | connecting | verifying | connected | error
        self.state = "disconnected"
        self.last_error = ""
        self.mega = {}                 # parsed from "ST k=v ..." lines
        self.mega_ts = 0.0
        self._connect_thread = None
        self.want_connected = True     # False only after an explicit Disconnect
        self.rx_bytes = 0              # every byte the Mega has ever sent us
        self._last_msg = ""            # for suppressing repeated failure spam
        self.attempts = 0

    def _say(self, msg):
        """Log, but never twice in a row for the same message - a Bluetooth
        module that is simply switched off would otherwise fill the console
        with the same line every few seconds."""
        if msg == self._last_msg:
            return
        self._last_msg = msg
        LOG.add("app", msg)

    # ---------------- lifecycle ----------------
    @property
    def connected(self):
        return self._ser is not None and self.state == "connected"

    def connect_async(self):
        if self.state in ("connecting", "verifying", "connected"):
            return
        if self._connect_thread is not None and self._connect_thread.is_alive():
            return
        self.want_connected = True
        self._connect_thread = threading.Thread(target=self._connect, daemon=True)
        self._connect_thread.start()

    def _connect(self):
        """Open the port AND prove there is a Mega on the other end.

        This two-step matters: 'rfcomm bind' only creates the /dev/rfcommN
        node - it says nothing about whether the HC-05 is powered, paired
        or in range. Opening that node therefore succeeds even with the
        robot switched off, and the failure only shows up on the first
        read as 'device reports readiness to read but returned no data'.
        Reporting 'connected' at open time would be a lie, so the link is
        not called connected until the Mega has actually answered.
        """
        if serial is None:
            self._fail("pyserial not installed - pip install pyserial")
            return

        if self.rfcomm is not None and self.rfcomm.enabled:
            if not self.rfcomm.link_connected():
                self._fail("no Bluetooth link to the HC-05 yet%s"
                           % ((" - " + self.rfcomm.last_error)
                              if self.rfcomm.last_error else ""))
                return

        self.attempts += 1
        self.state = "connecting"
        if self.attempts == 1:
            # Only announce the first attempt of a run. Retries alternating
            # "opening" / "never answered" would defeat the dedupe in _say
            # and refill the console anyway; the status panel keeps showing
            # the reason for as long as it applies.
            LOG.add("app", "opening %s @ %d ..." % (self.port, self.baud))
        try:
            s = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            self._fail("cannot open %s: %s" % (self.port, e))
            return

        self._ser = s
        self.state = "verifying"
        self._stop.clear()
        if self._reader is None or not self._reader.is_alive():
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()

        if not self._handshake():
            self._fail(self._diagnose_silence())
            return

        self.state = "connected"
        self.last_error = ""
        self._last_msg = ""
        self.attempts = 0
        LOG.add("app", "Mega connected on %s (replied to STATUS)" % self.port)

    def _handshake(self, timeout=None):
        timeout = HANDSHAKE_TIMEOUT if timeout is None else timeout
        """Poke the Mega and wait for it to say ANYTHING back. STATUS is the
        poke because the updated firmware answers with a status line and the
        old firmware answers '? STATUS' - either one proves the link."""
        start = self.rx_bytes
        time.sleep(0.4)                       # let the HC-05 settle
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        start = self.rx_bytes
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ser is None:             # reader already gave up
                return False
            self._raw_write("STATUS")
            for _ in range(15):
                if self.rx_bytes > start:
                    return True
                if self._ser is None:
                    return False
                time.sleep(0.1)
        return self.rx_bytes > start

    def _diagnose_silence(self):
        """Work out WHY nothing came back, and say so in the operator's terms.

        Three different faults all look like 'no reply' from here, and the
        fix for each is different, so they get three different messages."""
        radio = self.rfcomm.link_connected() if self.rfcomm else None
        drop = self.last_error

        if radio is False:
            return ("no Bluetooth link to the HC-05 on %s - the module is not "
                    "connected (powered off, out of range, or the pairing was "
                    "lost). %s" % (self.port,
                                   (self.rfcomm.last_error if self.rfcomm else "")))
        if radio is True:
            return ("Bluetooth is CONNECTED but the Mega sent nothing back in "
                    "%.0fs. The HC-05 having power only proves its radio is "
                    "alive - it says nothing about the board. Check: (1) the "
                    "Mega is actually powered and running, (2) TX1/D18 -> "
                    "HC-05 RXD and RX1/D19 -> HC-05 TXD are intact, (3) the "
                    "HC-05 shares a ground with the Mega."
                    % HANDSHAKE_TIMEOUT)
        return ("%s opened but nothing answered%s"
                % (self.port, (" (%s)" % drop) if drop else ""))

    def _fail(self, msg):
        """Give up on this attempt, closing the port and saying why once."""
        s, self._ser = self._ser, None
        self.state = "error"
        self.last_error = msg
        if s is not None:
            try:
                s.close()
            except Exception:
                pass
        self._say(msg)

    def disconnect(self):
        self.want_connected = False
        s, self._ser = self._ser, None
        self.state = "disconnected"
        self._last_msg = ""
        if s is not None:
            try:
                s.close()
            except Exception:
                pass
            LOG.add("app", "Mega disconnected")

    def _drop(self, why):
        """Called from the reader thread when a read/write blows up."""
        text = str(why)
        if "returned no data" in text:
            text = "no data from the Mega (powered off, out of range, or "
            text += "something else has %s open)" % self.port
        s, self._ser = self._ser, None
        was = self.state
        self.state = "error"
        self.last_error = text
        if s is not None:
            try:
                s.close()
            except Exception:
                pass
        # During the handshake this is the expected outcome for a robot that
        # is simply switched off - _connect reports it in plain language.
        if was == "connected":
            self._say("serial link lost: %s" % text)

    # ---------------- reader ----------------
    def _read_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            s = self._ser
            if s is None:
                time.sleep(0.2)
                continue
            try:
                chunk = s.read(1)
                if s.in_waiting:
                    chunk += s.read(s.in_waiting)
            except Exception as e:
                # If _ser has already moved on, this port was closed
                # deliberately (a failed handshake, a Disconnect) and the
                # resulting EBADF is an artefact of that, not a fault.
                # Reporting it would overwrite the real reason we closed.
                if self._ser is s:
                    self._drop(e)
                else:
                    buf = bytearray()
                continue
            if not chunk:
                continue
            self.rx_bytes += len(chunk)
            buf.extend(chunk)
            while True:
                idx = -1
                for i, b in enumerate(buf):
                    if b in (10, 13):
                        idx = i
                        break
                if idx < 0:
                    break
                line = bytes(buf[:idx]).decode("utf-8", "ignore").strip()
                del buf[:idx + 1]
                if line:
                    self._on_line(line)
            if len(buf) > 512:
                del buf[:-256]

    def _on_line(self, line):
        # STATUS replies are high-frequency housekeeping: parse them into
        # self.mega and keep them out of the operator console.
        if line.startswith("ST ") and "=" in line:
            self._parse_status(line)
            return
        LOG.add("mega", line)

    # "drive" is the single-char drive command and is legitimately "0" when
    # stopped - floating it would turn that into 0.0 and lose the meaning.
    STATUS_STR_KEYS = ("drive",)

    def _parse_status(self, line):
        d = {}
        for tok in line[3:].split():
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            if k in self.STATUS_STR_KEYS:
                d[k] = v
                continue
            try:
                d[k] = float(v)
            except ValueError:
                d[k] = v
        self.mega = d
        self.mega_ts = time.time()
        # Adopt the firmware's own keep-out rather than trusting a
        # constant here that could drift out of date after a KPX change.
        kpx = d.get("kpx")
        if isinstance(kpx, float) and abs(kpx - SET.keep_x) > 0.05:
            LOG.add("app", "bin keep-out: adopting keepX=%.0f from the Mega "
                           "(was %.0f)" % (kpx, SET.keep_x))
            SET.keep_x = kpx

    # ---------------- writers ----------------
    def _raw_write(self, text):
        s = self._ser
        if s is None:
            return False
        try:
            s.write((text.rstrip("\r\n") + "\n").encode())
            s.flush()
            return True
        except Exception as e:
            self._drop(e)
            return False

    def write_line(self, text, quiet=False, lock_timeout=5.0):
        """Fire and forget. Returns (ok, reason)."""
        if not self.connected:
            return False, "Mega not connected"
        if not self._tx.acquire(timeout=lock_timeout):
            return False, "serial busy"
        try:
            ok = self._raw_write(text)
        finally:
            self._tx.release()
        if ok and not quiet:
            LOG.add("tx", text)
        return ok, ("" if ok else "write failed")

    def interrupt(self, text="0"):
        """Bypasses the tx lock - emergency path only."""
        ok = self._raw_write(text)
        if ok:
            LOG.add("tx", "%s   <-- interrupt" % text)
        return ok

    def command(self, text, pred, timeout, quiet=False):
        """Write, then hold the link until pred(src, line) matches a Mega
        reply. Returns the matching line, or None on timeout."""
        if not self.connected:
            return None
        if not self._tx.acquire(timeout=5.0):
            return None
        try:
            mark = LOG.seq()
            if not self._raw_write(text):
                return None
            if not quiet:
                LOG.add("tx", text)
            return LOG.wait(lambda src, t: src == "mega" and pred(src, t),
                            timeout, mark)
        finally:
            self._tx.release()


# ======================================================================
#  VISION
# ----------------------------------------------------------------------
#  One capture+inference thread, many MJPEG clients. production.py ran
#  the whole pipeline inside the Flask generator, so a second browser tab
#  meant a second inference loop fighting for the same camera. Here the
#  thread publishes one annotated JPEG and every client just reads it.
# ======================================================================
class Vision:
    def __init__(self, model_path, cam_index, imgsz, width, height,
                 quality, calib_file):
        self.model_path = model_path
        self.cam_index = cam_index
        self.imgsz = imgsz
        self.width = width
        self.height = height
        self.quality = quality

        self.model = None
        self.cap = None
        self.M = None
        # Kept apart so a camera that comes back does not erase the fact
        # that the model failed to load, and vice versa.
        self.error = ""          # camera
        self.model_error = ""
        self.calib_error = ""

        self.fps = 0.0
        self.infer_ms = 0.0
        self.frames = 0
        self.cam_ok = False

        self.detection = None       # best in-reach target, or None
        self.det_count = 0
        self.last_seen_ts = 0.0     # last time an in-reach weed was visible

        self.jpeg = None
        self.jpeg_seq = 0
        # The newest frame BEFORE overlays are drawn. Calibration needs a
        # clean image, and demo.py is the process that wins the race for
        # this flaky camera - so it is the sensible place to get one from.
        self.raw_jpeg = None
        self.raw_ts = 0.0
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self._reach_cache = (None, None)
        self.status_provider = lambda: {}
        self._last_msg = ""
        self._retry_s = 5.0

        try:
            self.M = np.load(calib_file)
        except Exception as e:
            self.calib_error = "calibration: %s" % e
            LOG.add("app", "calibration load failed: %s" % e)

    def problem(self):
        return " | ".join(x for x in (self.model_error, self.error,
                                      self.calib_error) if x)

    # ---------------- startup ----------------
    def start(self):
        threading.Thread(target=self._boot, daemon=True).start()

    def _boot(self):
        try:
            from ultralytics import YOLO
            LOG.add("app", "loading model %s ..." % self.model_path)
            self.model = YOLO(self.model_path, task="detect")
            LOG.add("app", "model loaded")
        except Exception as e:
            self.model_error = "model: %s" % e
            LOG.add("app", "model load failed: %s" % e)

        self._open_camera()          # sets self.error itself on failure
        threading.Thread(target=self._loop, daemon=True).start()

    def _say(self, msg):
        """Same trick as SerialLink._say: an unplugged camera retried every
        few seconds should say so once, not scroll the console forever."""
        if msg == self._last_msg:
            return
        self._last_msg = msg
        LOG.add("app", msg)

    def _open_camera(self):
        try:
            # Ask for V4L2 explicitly. Left to choose, OpenCV falls through
            # to the FFMPEG backend and prints a misleading "should be
            # configured with libavdevice" warning when the real problem is
            # simply that /dev/video0 does not exist.
            if hasattr(cv2, "CAP_V4L2") and os.name == "posix":
                cap = cv2.VideoCapture(self.cam_index, cv2.CAP_V4L2)
            else:
                cap = cv2.VideoCapture(self.cam_index)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # not all backends honour this
            except Exception:
                pass
            if not cap.isOpened():
                dev = "/dev/video%d" % self.cam_index
                if os.name == "posix" and not os.path.exists(dev):
                    self._say("camera %d not found (%s does not exist) - "
                              "is the USB camera plugged in? retrying"
                              % (self.cam_index, dev))
                else:
                    self._say("camera %d exists but will not open - either "
                              "another process has it, or it is mid "
                              "re-enumeration (this camera drops off the USB "
                              "bus; check dmesg). retrying" % self.cam_index)
                self.error = "camera unavailable"
                return False
            self.cap = cap
            self.cam_ok = True
            self.error = ""
            self._last_msg = ""
            LOG.add("app", "camera %d open at %dx%d"
                    % (self.cam_index, self.width, self.height))
            # Applied on EVERY open, not just the first. A camera that
            # was unplugged and put back comes up at factory defaults -
            # gain 128, brightness 128, backlight compensation ON - which
            # is what makes the picture wash out. Controls apply live, so
            # this is cheap and idempotent.
            if camera_setup is not None:
                try:
                    camera_setup.apply_controls(
                        "/dev/video%d" % self.cam_index,
                        log=lambda m: LOG.add("app", m))
                except Exception as e:
                    LOG.add("app", "camera control setup failed: %s" % e)
            return True
        except Exception as e:
            self._say("camera open failed: %s" % e)
            return False

    def stop(self):
        self._stop.set()

    # ---------------- coordinate transform ----------------
    def pixel_to_mm(self, px, py):
        if self.M is None:
            return None, None
        pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
        out = cv2.perspectiveTransform(pt, self.M)
        return float(out[0][0][0]), float(out[0][0][1])

    def _reach_polygon(self, radius):
        """The SAFE_R circle, in robot mm, projected back into pixels so the
        operator can see which weeds the arm can actually get to."""
        if self.M is None:
            return None
        cached_r, cached_pts = self._reach_cache
        if cached_r == radius:
            return cached_pts
        try:
            inv = np.linalg.inv(self.M)
        except np.linalg.LinAlgError:
            return None
        ang = np.linspace(0.0, 2.0 * np.pi, 60, endpoint=False)
        ring = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
        ring = ring.reshape(-1, 1, 2).astype(np.float32)
        pts = cv2.perspectiveTransform(ring, inv).reshape(-1, 2).astype(np.int32)
        self._reach_cache = (radius, pts)
        return pts

    # ---------------- detection access ----------------
    def fresh_detection(self, after_ts=0.0):
        """The current target, but only if it is in reach, newer than
        after_ts, and not stale. The after_ts gate is what forces the
        auto loop to act on a frame captured AFTER the platform stopped,
        rather than on coordinates measured while it was still rolling."""
        d = self.detection
        if d is None:
            return None
        if not d["in_reach"]:
            return None
        if d["ts"] <= after_ts:
            return None
        if time.time() - d["ts"] > SET.det_max_age:
            return None
        return d

    # ---------------- main loop ----------------
    def _loop(self):
        prev = time.perf_counter()
        last_reopen = 0.0

        while not self._stop.is_set():
            if self.cap is None:
                self._publish(self._placeholder("NO CAMERA"))
                if time.time() - last_reopen > self._retry_s:
                    last_reopen = time.time()
                    if self._open_camera():
                        self._retry_s = 5.0
                    else:
                        # Same reasoning as the serial backoff: a camera
                        # that is not plugged in will not be plugged in
                        # any faster for being probed every 5 seconds.
                        self._retry_s = min(self._retry_s * 2, 30.0)
                time.sleep(0.5)
                continue

            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.cam_ok = False
                if time.time() - last_reopen > 2.0:
                    last_reopen = time.time()
                    self._say("camera read failed - reopening")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                time.sleep(0.05)
                continue
            self.cam_ok = True

            boxes = []
            t0 = time.perf_counter()
            if self.model is not None:
                try:
                    res = self.model.predict(frame, imgsz=self.imgsz,
                                             conf=SET.conf_thresh, verbose=False)
                    for b in res[0].boxes:
                        conf = float(b.conf[0])
                        cls_id = int(b.cls[0])
                        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                        mx, my = self.pixel_to_mm(cx, cy)
                        r = None if mx is None else float(np.hypot(mx, my))
                        boxes.append({
                            "label": self._class_name(cls_id),
                            "conf": conf,
                            "box": (x1, y1, x2, y2),
                            "px": cx, "py": cy,
                            "x_mm": mx, "y_mm": my, "r_mm": r,
                            "in_reach": (r is not None and r <= SET.safe_r
                                         and mx < SET.keep_x),
                        })
                except Exception as e:
                    LOG.add("app", "inference error: %s" % e)
                    time.sleep(0.2)
            t1 = time.perf_counter()

            # exponential smoothing - a raw per-frame number is unreadable
            self.infer_ms = self.infer_ms * 0.8 + (t1 - t0) * 1000.0 * 0.2

            # Target = highest-confidence box the arm can actually reach.
            reachable = [b for b in boxes if b["in_reach"]]
            best = max(reachable, key=lambda b: b["conf"]) if reachable else None
            if best is None and boxes:
                best = max(boxes, key=lambda b: b["conf"])   # shown, never picked

            now = time.time()
            self.det_count = len(boxes)
            if best is not None:
                self.detection = dict(best, ts=now)
                if best["in_reach"]:
                    self.last_seen_ts = now
            else:
                self.detection = None

            # Copy before drawing: _draw mutates the frame in place, and
            # a calibration photo with FPS text and bounding boxes burnt
            # into it is useless. ~1 ms per frame at this resolution.
            ok, rawjpg = cv2.imencode(".jpg", frame,
                                      [cv2.IMWRITE_JPEG_QUALITY, 95])
            if ok:
                self.raw_jpeg = rawjpg.tobytes()
                self.raw_ts = time.time()

            self._draw(frame, boxes, best)

            dt = time.perf_counter() - prev
            prev = time.perf_counter()
            if dt > 0:
                self.fps = self.fps * 0.8 + (1.0 / dt) * 0.2
            self.frames += 1

            ok, jpg = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                self._publish(jpg.tobytes())

    def _class_name(self, cls_id):
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            return names.get(cls_id, str(cls_id))
        if names:
            try:
                return names[cls_id]
            except Exception:
                pass
        return str(cls_id)

    # ---------------- overlays ----------------
    def _draw(self, frame, boxes, best):
        h, w = frame.shape[:2]

        if SET.show_reach:
            pts = self._reach_polygon(SET.safe_r)
            if pts is not None:
                cv2.polylines(frame, [pts], True, (255, 200, 0), 2)
                cv2.putText(frame, "reach r=%.0fmm" % SET.safe_r,
                            (int(pts[:, 0].min()), max(int(pts[:, 1].min()) - 8, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

        if SET.show_reach and self.M is not None:
            # The bin keep-out edge: everything at X >= keep_x is
            # unpickable, so draw where that line falls in the image.
            try:
                inv = np.linalg.inv(self.M)
                ys = np.linspace(-SET.safe_r, SET.safe_r, 24)
                edge = np.stack([np.full_like(ys, SET.keep_x), ys], axis=1)
                edge = edge.reshape(-1, 1, 2).astype(np.float32)
                ep = cv2.perspectiveTransform(edge, inv).reshape(-1, 2)
                ep = ep.astype(np.int32)
                cv2.polylines(frame, [ep], False, (0, 0, 255), 2)
                cv2.putText(frame, "bin keep-out X>=%.0f" % SET.keep_x,
                            (int(ep[len(ep) // 2][0]) + 6,
                             int(ep[len(ep) // 2][1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            except Exception:
                pass

        if SET.show_boxes:
            for b in boxes:
                x1, y1, x2, y2 = b["box"]
                target = (best is not None and b is best)
                if not b["in_reach"]:
                    col = (110, 110, 110)          # out of reach - greyed out
                elif target:
                    col = (0, 255, 0)
                else:
                    col = (0, 180, 120)
                cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3 if target else 1)
                cv2.putText(frame, "%s %.2f" % (b["label"], b["conf"]),
                            (x1, max(y1 - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, col, 2 if target else 1)
                if target:
                    cv2.circle(frame, (int(b["px"]), int(b["py"])), 5, (0, 0, 255), -1)
                    if b["x_mm"] is not None:
                        tag = "X%.1f Y%.1f" % (b["x_mm"], b["y_mm"])
                        if not b["in_reach"]:
                            tag += "  OUT OF REACH"
                        cv2.putText(frame, tag, (x1, min(y2 + 20, h - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

        try:
            st = self.status_provider() or {}
        except Exception:
            st = {}

        mode = str(st.get("mode", "?")).upper()
        arm = str(st.get("arm", "idle"))
        mega_ok = bool(st.get("mega", False))

        # translucent header strip so white text stays readable over soil
        band = frame[0:70, 0:w]
        cv2.addWeighted(band, 0.35, np.zeros_like(band), 0.65, 0, band)

        cv2.putText(frame, "FPS %.1f   INFER %.0f ms   DET %d" % (
                        self.fps, self.infer_ms, self.det_count),
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        mode_col = (0, 200, 255) if mode == "AUTO" else (200, 200, 200)
        cv2.putText(frame, "MODE %s" % mode, (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_col, 2)

        arm_col = (0, 165, 255) if arm not in ("idle", "off") else (0, 255, 0)
        cv2.putText(frame, "ARM %s" % arm.upper(), (190, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, arm_col, 2)

        cv2.putText(frame, "MEGA %s" % ("OK" if mega_ok else "DOWN"),
                    (max(w - 160, 400), 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if mega_ok else (0, 0, 255), 2)

    def _placeholder(self, msg):
        img = np.zeros((max(self.height // 2, 240), max(self.width // 2, 320), 3),
                       dtype=np.uint8)
        cv2.putText(img, msg, (30, img.shape[0] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        prob = self.problem()
        if prob:
            cv2.putText(img, prob[:70], (30, img.shape[0] // 2 + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        ok, jpg = cv2.imencode(".jpg", img)
        return jpg.tobytes() if ok else b""

    # ---------------- publish / consume ----------------
    def _publish(self, data):
        if not data:
            return
        with self._cv:
            self.jpeg = data
            self.jpeg_seq += 1
            self._cv.notify_all()

    def frames_iter(self):
        last = -1
        while True:
            with self._cv:
                self._cv.wait_for(lambda: self.jpeg_seq != last, timeout=2.0)
                if self.jpeg_seq == last or self.jpeg is None:
                    continue
                last = self.jpeg_seq
                data = self.jpeg
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")

    def snapshot(self):
        with self._cv:
            return self.jpeg


# ======================================================================
#  AUTO WEEDER
# ----------------------------------------------------------------------
#  The loop you asked for, as an explicit state machine:
#
#    SEARCH   wait up to search_timeout for an in-reach weed.
#             found -> PICK.  nothing -> ADVANCE.
#
#    ADVANCE  send F, refresh it every drive_refresh_s (the Mega's own
#             watchdog stops the wheels 5 s after the last command, so
#             the refresh is also the deadman). The moment an in-reach
#             weed appears: send 0 from HERE, not from the Mega, so the
#             platform is already stopped before any coordinate is
#             trusted. Then settle, then REACQUIRE.
#
#    REACQUIRE  demand a detection from a frame captured strictly AFTER
#             the stop settled. That is the whole point of stopping from
#             the Pi: the coordinate that gets picked was measured with
#             the wheels already still. Lost it -> back to ADVANCE.
#
#    PICK     "PICK X.. Y..", block for DONE / ERR / timeout, repeat.
#
#  The Mega also stops the wheels itself the instant it accepts a PICK
#  (doPick calls stopAll), which stays in place as a backstop.
# ======================================================================
class AutoWeeder:
    def __init__(self, link, vision, controller):
        self.link = link
        self.vision = vision
        self.ctl = controller
        self._thread = None
        self._stop = threading.Event()
        self.state = "off"
        self.driving = False
        self.picks_ok = 0
        self.picks_err = 0
        self.last_result = ""
        self.last_pick = None       # dict with x/y/result/ts

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def request_stop(self):
        self._stop.set()

    def join(self, timeout):
        if self._thread is not None:
            self._thread.join(timeout)

    # ---------------- the loop ----------------
    def _run(self):
        LOG.add("app", "AUTO: started")
        self.state = "search"
        try:
            while not self._stop.is_set():
                if not self.link.connected:
                    self.state = "waiting for mega"
                    time.sleep(0.5)
                    continue

                after = time.time()
                det = self._wait_for_weed(SET.search_timeout, after)

                if det is None and not self._stop.is_set():
                    det = self._advance_until_weed()

                if det is None:
                    continue

                self._do_pick(det)
        except Exception as e:
            LOG.add("app", "AUTO crashed: %s" % e)
        finally:
            self._drive_stop()
            self.state = "off"
            LOG.add("app", "AUTO: stopped")

    def _wait_for_weed(self, timeout, after_ts):
        """Poll the vision thread for an in-reach detection from a frame
        newer than after_ts. Returns the detection or None."""
        self.state = "search"
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._stop.is_set():
                return None
            det = self.vision.fresh_detection(after_ts)
            if det is not None:
                return det
            time.sleep(0.03)
        return None

    def _advance_until_weed(self):
        self.state = "advance"
        LOG.add("app", "AUTO: no weed for %.1fs - driving forward" % SET.search_timeout)
        started = time.time()
        last_drive = 0.0
        try:
            while not self._stop.is_set():
                if time.time() - started > SET.advance_max_s:
                    self._drive_stop()
                    self.state = "advance timeout"
                    LOG.add("app", "AUTO: drove %.0fs with no weed - pausing"
                            % SET.advance_max_s)
                    time.sleep(1.5)
                    return None

                if time.time() - last_drive > SET.drive_refresh_s:
                    ok, why = self.link.write_line("F", quiet=True)
                    if not ok:
                        LOG.add("app", "AUTO: drive failed (%s)" % why)
                        return None
                    self.driving = True
                    last_drive = time.time()

                if self.vision.fresh_detection(0.0) is not None:
                    # Weed in reach while rolling. Stop from the Pi FIRST,
                    # then re-measure - the coordinates seen mid-roll are
                    # already out of date by the time the arm would move.
                    self._drive_stop()
                    self.state = "settle"
                    time.sleep(SET.settle_s)

                    self.state = "reacquire"
                    stopped_at = time.time()
                    det = self._wait_for_weed(SET.reacquire_s, stopped_at)
                    if det is not None:
                        LOG.add("app", "AUTO: reacquired at X%.1f Y%.1f after stop"
                                % (det["x_mm"], det["y_mm"]))
                        return det

                    LOG.add("app", "AUTO: lost it after stopping - resuming drive")
                    self.state = "advance"
                    last_drive = 0.0
                    continue

                time.sleep(0.03)
        finally:
            self._drive_stop()
        return None

    def _do_pick(self, det):
        x, y = det["x_mm"], det["y_mm"]
        self.state = "picking"
        self.ctl.set_pick_busy(True)
        try:
            line = self.link.command(
                "PICK X%.1f Y%.1f" % (x, y),
                lambda src, t: t == "DONE" or t.startswith("ERR"),
                SET.pick_timeout_s)
        finally:
            self.ctl.set_pick_busy(False)

        result = line if line else "TIMEOUT"
        self.last_result = result
        self.last_pick = {"x": x, "y": y, "result": result, "ts": time.time()}

        if result == "DONE":
            self.picks_ok += 1
        else:
            self.picks_err += 1
            LOG.add("app", "AUTO: pick returned %s" % result)
            if result.startswith("ERR BUSY") or result == "TIMEOUT":
                time.sleep(1.0)       # let the Mega catch up before retrying

        self.state = "search"
        time.sleep(SET.post_pick_s)

    def _drive_stop(self):
        if self.driving or self.link.connected:
            self.link.write_line("0", quiet=True)
        self.driving = False


# ======================================================================
#  CONTROLLER  - mode arbitration
# ----------------------------------------------------------------------
#  Exactly one owner of the Mega at a time:
#
#    MANUAL  the web UI owns it. The auto thread is not running.
#    AUTO    the auto thread owns it. Manual endpoints are refused with
#            409 and a reason the UI shows, EXCEPT the emergency stop,
#            which is always allowed and always wins.
#
#  pick_busy is a second, finer gate: true from the moment PICK goes out
#  until DONE/ERR comes back. Nothing manual may go down the wire in that
#  window - the "no manual jog mid-pick-cycle" guarantee. Because
#  SerialLink.command() holds the tx lock for the same window, it is
#  enforced twice: once as a friendly refusal in the API, once as a hard
#  lock on the port.
#
#  Leaving AUTO is graceful: the auto thread is asked to stop, finishes
#  the pick it is in (an arm abandoned mid-cycle is holding a weed over
#  open ground), then exits and the mode flips. Impatient operators have
#  the E-STOP button, which aborts the move outright.
# ======================================================================
MANUAL = "manual"
AUTO = "auto"


class Controller:
    def __init__(self, link, vision):
        self.link = link
        self.vision = vision
        self.mode = MANUAL
        self.pick_busy = False
        self.switching = False
        self.auto = AutoWeeder(link, vision, self)
        self._lock = threading.RLock()
        self.estop_ts = 0.0

    # ---------------- gates ----------------
    def set_pick_busy(self, v):
        self.pick_busy = bool(v)

    def manual_gate(self):
        """(ok, reason) - may a manual command go out right now?"""
        if self.switching:
            return False, "mode switch in progress"
        if self.mode == AUTO:
            return False, "autoweeding is ON - switch to manual first"
        if self.pick_busy:
            return False, "pick cycle in progress"
        if not self.link.connected:
            return False, "Mega not connected"
        return True, ""

    # ---------------- mode ----------------
    def set_mode(self, mode):
        with self._lock:
            if mode not in (MANUAL, AUTO):
                return False, "unknown mode"
            if mode == self.mode and not self.switching:
                return True, ""
            if mode == AUTO:
                if not self.link.connected:
                    return False, "Mega not connected"
                self.mode = AUTO
                self.switching = False
                self.auto.start()
                LOG.add("app", "mode -> AUTO (autoweeding on)")
                return True, ""
            # AUTO -> MANUAL, off the request thread so the UI stays live
            self.switching = True
            LOG.add("app", "mode -> MANUAL requested (finishing current cycle)")
            self.auto.request_stop()
            threading.Thread(target=self._finish_switch, daemon=True).start()
            return True, ""

    def _finish_switch(self):
        self.auto.join(SET.pick_timeout_s + 5.0)
        with self._lock:
            self.mode = MANUAL
            self.switching = False
        self.link.write_line("0", quiet=True)
        LOG.add("app", "mode -> MANUAL")

    # ---------------- emergency ----------------
    def estop(self):
        """Always allowed, from any mode, at any point in a cycle."""
        self.estop_ts = time.time()
        was_running = self.auto.running
        self.auto.request_stop()
        self.link.interrupt("0")          # abort delta move + stop wheels
        self.mode = MANUAL
        if was_running:
            self.switching = True
            threading.Thread(target=self._finish_switch, daemon=True).start()
        LOG.add("app", "*** E-STOP ***")
        return True, ""

    # ---------------- manual actions ----------------
    def manual_send(self, line, expect=None, timeout=6.0):
        ok, why = self.manual_gate()
        if not ok:
            return False, why, None
        if expect is None:
            sent, why = self.link.write_line(line)
            return sent, why, None
        reply = self.link.command(line, expect, timeout)
        if reply is None:
            return False, "no reply from Mega", None
        if reply.startswith("ERR"):
            return False, reply, reply
        return True, "", reply

    def jog(self, axis, delta):
        axis = str(axis).upper()
        if axis not in ("X", "Y", "Z"):
            return False, "bad axis", None
        return self.manual_send(
            "JOG%s%.2f" % (axis, float(delta)),
            lambda src, t: t.startswith("OK JOG") or t.startswith("ERR"),
            timeout=20.0)

    # ---------------- status ----------------
    def arm_state(self):
        """One word for what the arm/robot is doing, shared by the JSON
        status and the video overlay so they can never disagree."""
        if self.pick_busy:
            return "picking"
        if self.auto.driving:
            return "driving"
        if self.auto.running:
            return self.auto.state
        return "idle"

    def overlay_status(self):
        return {"mode": self.mode, "arm": self.arm_state(),
                "mega": self.link.connected}

    def status(self):
        det = self.vision.detection
        mega_fresh = (time.time() - self.link.mega_ts) < 4.0
        arm = self.arm_state()

        return {
            "mode": self.mode,
            "switching": self.switching,
            "auto_running": self.auto.running,
            "auto_state": self.auto.state,
            "pick_busy": self.pick_busy,
            "driving": self.auto.driving,
            "arm": arm,
            "manual_ok": self.manual_gate()[0],
            "manual_block": self.manual_gate()[1],
            "serial": {
                "state": self.link.state,
                "port": self.link.port,
                "connected": self.link.connected,
                "error": self.link.last_error,
            },
            # The radio link and the Mega behind it are reported separately
            # on purpose - "Bluetooth up, Mega silent" is a real and very
            # different state from "Bluetooth down".
            "radio": self.link.rfcomm.status() if self.link.rfcomm else None,
            "mega": dict(self.link.mega) if mega_fresh else {},
            "mega_fresh": mega_fresh,
            "vision": {
                "fps": round(self.vision.fps, 1),
                "infer_ms": round(self.vision.infer_ms, 1),
                "frames": self.vision.frames,
                "cam_ok": self.vision.cam_ok,
                "det_count": self.vision.det_count,
                "error": self.vision.problem(),
                "calibrated": self.vision.M is not None,
            },
            "detection": None if det is None else {
                "label": det["label"],
                "conf": round(det["conf"], 3),
                "px": round(det["px"], 1),
                "py": round(det["py"], 1),
                "x_mm": None if det["x_mm"] is None else round(det["x_mm"], 1),
                "y_mm": None if det["y_mm"] is None else round(det["y_mm"], 1),
                "r_mm": None if det["r_mm"] is None else round(det["r_mm"], 1),
                "in_reach": det["in_reach"],
                "age": round(time.time() - det["ts"], 2),
            },
            "picks": {
                "ok": self.auto.picks_ok,
                "err": self.auto.picks_err,
                "last": self.auto.last_pick,
            },
            "settings": SET.snapshot(),
            "ts": time.time(),
        }


# ======================================================================
#  BACKGROUND HOUSEKEEPING
# ----------------------------------------------------------------------
#  Reconnects the Mega on its own (the HC-05 drops if the robot is power
#  cycled) and polls STATUS whenever the link is idle, so the web UI
#  shows real firmware state - homed, position, drive command - instead
#  of guessing from whatever the Pi last sent.
#
#  The poll is skipped whenever a pick is in flight: the firmware
#  discards incoming bytes while pickBusy is set, so a STATUS sent then
#  would be swallowed anyway.
# ======================================================================
RECONNECT_MIN = 4.0
RECONNECT_MAX = 30.0


def housekeeping(link, ctl, auto_connect=True):
    last_status = 0.0
    next_try = 0.0
    backoff = RECONNECT_MIN
    while True:
        try:
            if link.connected:
                backoff = RECONNECT_MIN
                next_try = 0.0
            elif (auto_connect and link.want_connected
                    and link.state in ("disconnected", "error")):
                # Back off instead of retrying every 2 s: if the robot is
                # simply switched off, a tight retry loop achieves nothing
                # except burying every other message in the console.
                now = time.time()
                if now >= next_try:
                    if next_try and backoff < RECONNECT_MAX:
                        backoff = min(backoff * 2, RECONNECT_MAX)
                    next_try = now + backoff
                    # The radio has to be up before the tty is worth
                    # opening, and "rfcomm connect" only holds the link
                    # while its process lives - so re-raise it first if
                    # the robot was power-cycled.
                    if link.rfcomm is not None and link.rfcomm.enabled:
                        if not link.rfcomm.alive():
                            link.rfcomm.start()
                    link.connect_async()

            if (link.connected and not ctl.pick_busy
                    and time.time() - last_status > 1.0):
                last_status = time.time()
                # lock_timeout is tiny on purpose: if the link is mid
                # command, skip this tick rather than queue behind it.
                link.write_line("STATUS", quiet=True, lock_timeout=0.05)
        except Exception as e:
            LOG.add("app", "housekeeping: %s" % e)
        time.sleep(0.4)


# ======================================================================
#  WEB UI  (single self-contained page - the Pi may have no internet,
#  so there is no CDN, no framework, no build step)
# ======================================================================
# Raw string: the JavaScript below contains regex escapes such as \*\*
# which Python would otherwise try to interpret (SyntaxWarning).
PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Weed Robot Control</title>
<style>
:root{
  --bg:#0e1116; --panel:#161b22; --panel2:#1c2230; --line:#2a3240;
  --txt:#e6edf3; --dim:#8b96a5; --acc:#3fb950; --warn:#d29922;
  --bad:#f85149; --auto:#2f81f7;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);
  font:14px/1.45 ui-sans-serif,system-ui,"Segoe UI",Roboto,sans-serif}
header{display:flex;align-items:center;gap:14px;flex-wrap:wrap;
  padding:10px 16px;background:var(--panel);border-bottom:1px solid var(--line);
  position:sticky;top:0;z-index:10}
header h1{font-size:15px;margin:0;font-weight:650;letter-spacing:.3px}
.badge{padding:3px 10px;border-radius:999px;font-size:12px;font-weight:600;
  border:1px solid var(--line);background:var(--panel2);white-space:nowrap}
.badge.ok{color:var(--acc);border-color:#1f4a2a}
.badge.bad{color:var(--bad);border-color:#5c1f1c}
.badge.warn{color:var(--warn);border-color:#5c4611}
.badge.auto{color:#fff;background:var(--auto);border-color:var(--auto)}
.spacer{flex:1}
main{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(340px,1fr);
  gap:14px;padding:14px;align-items:start}
@media(max-width:1000px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;
  padding:12px;margin-bottom:14px}
.card h2{margin:0 0 10px;font-size:12px;text-transform:uppercase;
  letter-spacing:.9px;color:var(--dim);font-weight:700}
#stream{width:100%;display:block;border-radius:8px;background:#000}
.row{display:flex;gap:8px;flex-wrap:wrap}
button{background:var(--panel2);color:var(--txt);border:1px solid var(--line);
  border-radius:7px;padding:8px 12px;font-size:13px;cursor:pointer;
  font-family:inherit;transition:.12s}
button:hover:not(:disabled){background:#28303f;border-color:#3d4757}
button:active:not(:disabled){transform:translateY(1px)}
button:disabled{opacity:.38;cursor:not-allowed}
button.big{flex:1;padding:12px;font-weight:650;font-size:14px}
button.on{background:var(--auto);border-color:var(--auto);color:#fff}
button.stop{background:#5c1f1c;border-color:var(--bad);color:#ffb4ae;
  font-weight:700}
button.stop:hover{background:var(--bad);color:#fff}
button.ghost{background:transparent}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:270px}
.pad button{padding:14px 0;font-weight:650}
.pad .mid{background:#3a2020;border-color:#6b3030;color:#ffb4ae}
kbd{background:#000;border:1px solid var(--line);border-radius:4px;
  padding:0 4px;font-size:10px;color:var(--dim)}
</style>
</head>
<body>
<style>
table.kv{width:100%;border-collapse:collapse;font-size:13px}
table.kv td{padding:3px 0;vertical-align:top}
table.kv td:first-child{color:var(--dim);width:44%;white-space:nowrap}
table.kv td:last-child{text-align:right;font-variant-numeric:tabular-nums}
.sub{font-size:11px;color:var(--dim);margin:-4px 0 8px}
fieldset{border:1px solid var(--line);border-radius:8px;padding:9px 10px;
  margin:0 0 10px}
fieldset legend{font-size:11px;color:var(--dim);text-transform:uppercase;
  letter-spacing:.7px;padding:0 5px}
.jog{display:grid;grid-template-columns:34px 1fr 1fr;gap:7px;align-items:center}
.jog span{color:var(--dim);font-weight:700;text-align:center}
input,select{background:#0b0f14;color:var(--txt);border:1px solid var(--line);
  border-radius:6px;padding:7px 9px;font:13px ui-monospace,Menlo,Consolas,monospace;
  width:100%}
input:focus,select:focus{outline:none;border-color:var(--auto)}
.setgrid{display:grid;grid-template-columns:1fr 92px;gap:7px 10px;
  align-items:center;font-size:12.5px}
.setgrid label{color:var(--dim)}
#console{height:230px;overflow:auto;background:#0b0f14;border:1px solid var(--line);
  border-radius:7px;padding:8px;font:12px/1.5 ui-monospace,Menlo,Consolas,monospace;
  white-space:pre-wrap;word-break:break-word}
#console div{padding:1px 0}
.s-app{color:#79c0ff}.s-tx{color:var(--warn)}.s-mega{color:var(--acc)}
.s-err{color:var(--bad)}
#toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%);
  background:#5c1f1c;color:#fff;border:1px solid var(--bad);padding:9px 16px;
  border-radius:8px;font-size:13px;opacity:0;pointer-events:none;
  transition:opacity .2s;z-index:50;max-width:80vw}
#toast.show{opacity:1}
.chk{display:flex;align-items:center;gap:7px;font-size:12.5px;color:var(--dim)}
.chk input{width:auto}
</style>

<header>
  <h1>WEED ROBOT</h1>
  <span class="badge" id="b-mode">MODE ...</span>
  <span class="badge" id="b-bt">BT ...</span>
  <span class="badge" id="b-mega">MEGA ...</span>
  <span class="badge" id="b-arm">ARM ...</span>
  <span class="badge" id="b-det">DET ...</span>
  <span class="spacer"></span>
  <button class="stop" onclick="estop()">EMERGENCY STOP</button>
</header>

<main>
<section>
  <div class="card">
    <h2>Live detection</h2>
    <img id="stream" src="/stream" alt="stream">
    <p class="sub" id="vision-line" style="margin-top:8px">&nbsp;</p>
    <div class="row">
      <label class="chk"><input type="checkbox" id="ov-boxes" checked
        onchange="pushSettings()"> detection boxes</label>
      <label class="chk"><input type="checkbox" id="ov-reach" checked
        onchange="pushSettings()"> reach circle</label>
      <span class="spacer"></span>
      <button class="ghost" onclick="document.getElementById('stream').src='/stream?'+Date.now()">
        reload stream</button>
    </div>
  </div>

  <div class="card">
    <h2>Console</h2>
    <div id="console"></div>
    <div class="row" style="margin-top:9px">
      <input id="raw" placeholder="raw command to the Mega, e.g.  POS   LIMITS   GA150"
        autocomplete="off" style="flex:1;min-width:180px"
        onkeydown="if(event.key==='Enter')sendRaw()">
      <button onclick="sendRaw()">Send</button>
      <button class="ghost" onclick="document.getElementById('console').innerHTML=''">Clear</button>
    </div>
    <p class="sub" style="margin-top:7px">Raw commands obey the same gate as
      every other manual control. <kbd>0</kbd> and <kbd>STOP</kbd> always go
      through as an emergency stop.</p>
  </div>
</section>
<section>
  <div class="card">
    <h2>Mode</h2>
    <div class="row">
      <button class="big" id="btn-manual" onclick="setMode('manual')">MANUAL</button>
      <button class="big" id="btn-auto" onclick="setMode('auto')">AUTOWEEDING</button>
    </div>
    <p class="sub" id="mode-note" style="margin-top:9px">&nbsp;</p>
  </div>

  <div class="card">
    <h2>Status</h2>
    <table class="kv" id="kv"></table>
  </div>

  <div class="card">
    <h2>Platform</h2>
    <p class="sub">Hold a button (or <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>)
      to drive, release to stop. <kbd>Space</kbd> = stop.</p>
    <div class="pad">
      <span></span>
      <button data-drive="F">FWD</button>
      <span></span>
      <button data-drive="L">LEFT</button>
      <button class="mid" onclick="drive('0')">STOP</button>
      <button data-drive="R">RIGHT</button>
      <span></span>
      <button data-drive="B">BACK</button>
      <span></span>
    </div>
    <div class="row" style="margin-top:9px">
      <button onclick="send('/api/drive',{cmd:'T'})">speed +</button>
      <button onclick="send('/api/drive',{cmd:'X'})">speed -</button>
    </div>
  </div>

  <div class="card">
    <h2>Arm</h2>
    <fieldset><legend>power</legend>
      <div class="row">
        <button onclick="arm('enable')">Attach all (E)</button>
        <button onclick="arm('release')">Release all (D)</button>
      </div>
      <div class="row" style="margin-top:7px">
        <button onclick="arm('steppers_on')">Steppers ON</button>
        <button onclick="arm('steppers_off')">Steppers OFF</button>
        <button onclick="arm('servo_attach')">Servo att</button>
        <button onclick="arm('servo_detach')">Servo det</button>
      </div>
    </fieldset>
    <fieldset><legend>reference &amp; positions</legend>
      <div class="row">
        <button onclick="arm('sethome')">SET HOME</button>
        <button onclick="arm('unhome')">UNHOME</button>
        <button onclick="arm('home')">Go HOME</button>
      </div>
      <div class="row" style="margin-top:7px">
        <button onclick="arm('work')">WORK</button>
        <button onclick="arm('bin')">BIN</button>
        <button onclick="arm('park')">PARK</button>
        <button onclick="arm('pos')">POS</button>
        <button onclick="arm('limits')">LIMITS</button>
      </div>
    </fieldset>
    <fieldset><legend>gripper</legend>
      <div class="row">
        <button onclick="arm('grip_open')">Open</button>
        <button onclick="arm('grip_close')">Close</button>
        <button onclick="arm('grip_plus')">Nudge +</button>
        <button onclick="arm('grip_minus')">Nudge -</button>
      </div>
    </fieldset>
    <fieldset><legend>cartesian jog (mm)</legend>
      <div class="row" style="margin-bottom:8px">
        <select id="jogstep" onchange="pushSettings()" style="max-width:120px">
          <option value="1">1 mm</option>
          <option value="5">5 mm</option>
          <option value="10" selected>10 mm</option>
          <option value="25">25 mm</option>
        </select>
        <span class="sub" style="margin:0;align-self:center">needs SET HOME first</span>
      </div>
      <div class="jog">
        <span>X</span><button onclick="jog('X',-1)">- X</button><button onclick="jog('X',1)">+ X</button>
        <span>Y</span><button onclick="jog('Y',-1)">- Y</button><button onclick="jog('Y',1)">+ Y</button>
        <span>Z</span><button onclick="jog('Z',-1)">- Z (down)</button><button onclick="jog('Z',1)">+ Z (up)</button>
      </div>
    </fieldset>
    <fieldset><legend>manual pick test</legend>
      <div class="row">
        <input id="pick-x" placeholder="X mm" style="max-width:90px">
        <input id="pick-y" placeholder="Y mm" style="max-width:90px">
        <button onclick="manualPick()">PICK</button>
        <button onclick="pickDetected()">Pick current detection</button>
      </div>
    </fieldset>
  </div>
  <div class="card">
    <h2>Link</h2>
    <div class="row">
      <button onclick="send('/api/serial',{action:'connect'})">Connect</button>
      <button onclick="send('/api/serial',{action:'disconnect'})">Disconnect</button>
      <button onclick="send('/api/serial',{action:'reconnect'})">Reconnect Bluetooth</button>
    </div>
    <p class="sub" id="link-note" style="margin-top:8px">&nbsp;</p>
  </div>

  <div class="card">
    <h2>Tuning</h2>
    <div class="setgrid">
      <label>confidence threshold</label><input id="s-conf" type="number" step="0.05" min="0.05" max="0.95">
      <label>reach radius (mm)</label><input id="s-safe" type="number" step="5" min="10" max="200">
      <label>search timeout (s)</label><input id="s-search" type="number" step="0.5" min="0.5">
      <label>stop settle (s)</label><input id="s-settle" type="number" step="0.05" min="0">
      <label>re-acquire window (s)</label><input id="s-reacq" type="number" step="0.1" min="0.1">
      <label>drive refresh (s)</label><input id="s-drv" type="number" step="0.1" min="0.2" max="4">
      <label>max drive burst (s)</label><input id="s-adv" type="number" step="1" min="2">
      <label>after-pick pause (s)</label><input id="s-post" type="number" step="0.05" min="0">
      <label>pick timeout (s)</label><input id="s-ptmo" type="number" step="1" min="3">
    </div>
    <div class="row" style="margin-top:10px">
      <button onclick="pushSettings()">Apply</button>
    </div>
    <p class="sub" style="margin-top:8px">Applied live. Auto-weeding picks
      these up on the next loop iteration - no restart.</p>
  </div>
</section>
</main>

<div id="toast"></div>

<script>
let logSeq = 0, held = null, holdTimer = null, lastStatus = null;

function toast(msg){
  const t = document.getElementById('toast');
  t.textContent = msg; t.classList.add('show');
  clearTimeout(t._h); t._h = setTimeout(()=>t.classList.remove('show'), 2600);
}

async function send(url, body){
  try{
    const r = await fetch(url, {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body||{})});
    const j = await r.json().catch(()=>({}));
    if(!r.ok || j.ok === false) toast(j.error || ('HTTP ' + r.status));
    return j;
  }catch(e){ toast('request failed: ' + e); return {ok:false}; }
}

const setMode  = m => send('/api/mode', {mode:m});
const estop    = () => send('/api/estop', {});
const arm      = a => send('/api/arm', {action:a});
const drive    = c => send('/api/drive', {cmd:c});
const jog      = (ax,sign) => send('/api/jog',
  {axis:ax, delta: sign * parseFloat(document.getElementById('jogstep').value)});

function sendRaw(){
  const el = document.getElementById('raw');
  const v = el.value.trim();
  if(!v) return;
  send('/api/raw', {line:v});
  el.value = '';
}

function manualPick(){
  const x = parseFloat(document.getElementById('pick-x').value);
  const y = parseFloat(document.getElementById('pick-y').value);
  if(isNaN(x) || isNaN(y)){ toast('enter X and Y in mm'); return; }
  send('/api/pick', {x:x, y:y});
}

function pickDetected(){
  const d = lastStatus && lastStatus.detection;
  if(!d || d.x_mm === null){ toast('no detection right now'); return; }
  if(!d.in_reach){ toast('that detection is out of reach'); return; }
  send('/api/pick', {x:d.x_mm, y:d.y_mm});
}

/* ---- hold-to-drive -------------------------------------------------
   The Mega stops the wheels 5 s after the last command it heard, so a
   dropped mouseup or a closed tab cannot leave the platform running.
   While a button is held we refresh the command inside that window. */
function startHold(cmd){
  if(held === cmd) return;
  held = cmd; drive(cmd);
  clearInterval(holdTimer);
  holdTimer = setInterval(()=>drive(cmd), 1200);
}
function endHold(){
  if(held === null) return;
  held = null; clearInterval(holdTimer); drive('0');
}
document.querySelectorAll('[data-drive]').forEach(b=>{
  const c = b.dataset.drive;
  b.addEventListener('mousedown', e=>{e.preventDefault(); startHold(c);});
  b.addEventListener('touchstart', e=>{e.preventDefault(); startHold(c);}, {passive:false});
});
['mouseup','mouseleave','touchend','touchcancel','blur']
  .forEach(ev=>window.addEventListener(ev, endHold));

const KEYS = {w:'F', s:'B', a:'L', d:'R'};
addEventListener('keydown', e=>{
  if(e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if(e.repeat) return;
  const k = e.key.toLowerCase();
  if(KEYS[k]){ e.preventDefault(); startHold(KEYS[k]); }
  else if(e.key === ' '){ e.preventDefault(); endHold(); drive('0'); }
  else if(e.key === 'Escape'){ estop(); }
});
addEventListener('keyup', e=>{
  const k = e.key.toLowerCase();
  if(KEYS[k] && held === KEYS[k]) endHold();
});

/* ---- settings ---- */
const SFIELDS = {
  's-conf':'conf_thresh', 's-safe':'safe_r', 's-search':'search_timeout',
  's-settle':'settle_s', 's-reacq':'reacquire_s', 's-drv':'drive_refresh_s',
  's-adv':'advance_max_s', 's-post':'post_pick_s', 's-ptmo':'pick_timeout_s'
};
let settingsLoaded = false;

function pushSettings(){
  const body = {
    show_boxes: document.getElementById('ov-boxes').checked,
    show_reach: document.getElementById('ov-reach').checked,
    jog_step: parseFloat(document.getElementById('jogstep').value)
  };
  for(const [id,key] of Object.entries(SFIELDS)){
    const v = parseFloat(document.getElementById(id).value);
    if(!isNaN(v)) body[key] = v;
  }
  send('/api/settings', body);
}

function fillSettings(s){
  if(settingsLoaded) return;
  settingsLoaded = true;
  for(const [id,key] of Object.entries(SFIELDS))
    document.getElementById(id).value = s[key];
  document.getElementById('ov-boxes').checked = s.show_boxes;
  document.getElementById('ov-reach').checked = s.show_reach;
}

/* ---- status ---- */
function badge(el, text, cls){
  el.textContent = text;
  el.className = 'badge' + (cls ? ' ' + cls : '');
}

function row(k, v){ return '<tr><td>' + k + '</td><td>' + v + '</td></tr>'; }

async function poll(){
  let s;
  try{ s = await (await fetch('/api/status')).json(); }
  catch(e){
    badge(document.getElementById('b-mega'), 'PI UNREACHABLE', 'bad');
    return;
  }
  lastStatus = s;
  fillSettings(s.settings);

  const auto = s.mode === 'auto';
  badge(document.getElementById('b-mode'),
        s.switching ? 'SWITCHING...' : 'MODE ' + s.mode.toUpperCase(),
        s.switching ? 'warn' : (auto ? 'auto' : ''));
  /* The radio and the board behind it are two different things, and
     "BT up / MEGA down" is the exact state that means check the wiring
     rather than check the pairing. Show them as two badges. */
  const radio = s.radio;
  badge(document.getElementById('b-bt'),
        !radio ? 'BT n/a' : (radio.connected ? 'BT LINKED' : 'BT DOWN'),
        !radio ? '' : (radio.connected ? 'ok' : 'bad'));
  badge(document.getElementById('b-mega'),
        'MEGA ' + s.serial.state.toUpperCase(),
        s.serial.connected ? 'ok'
          : (['connecting','verifying'].includes(s.serial.state) ? 'warn' : 'bad'));
  badge(document.getElementById('b-arm'), 'ARM ' + s.arm.toUpperCase(),
        s.pick_busy ? 'warn' : (s.arm === 'idle' ? 'ok' : 'warn'));

  const d = s.detection;
  badge(document.getElementById('b-det'),
        d ? ('DET ' + d.label + ' ' + d.conf.toFixed(2) +
             (d.x_mm === null ? '' : '  X' + d.x_mm + ' Y' + d.y_mm) +
             (d.in_reach ? '' : '  OUT OF REACH'))
          : 'NO DETECTION',
        d ? (d.in_reach ? 'ok' : 'warn') : '');

  document.getElementById('btn-auto').className   = 'big' + (auto ? ' on' : '');
  document.getElementById('btn-manual').className = 'big' + (auto ? '' : ' on');
  document.getElementById('btn-auto').disabled    = s.switching;
  document.getElementById('btn-manual').disabled  = s.switching;
  document.getElementById('mode-note').textContent =
    s.switching ? 'Finishing the current pick cycle before handing control back...'
    : auto ? ('Auto state: ' + s.auto_state + '. Manual controls are locked out.')
    : (s.manual_ok ? 'Manual control active.' : 'Manual blocked: ' + s.manual_block);

  document.getElementById('vision-line').textContent =
    'FPS ' + s.vision.fps + '   inference ' + s.vision.infer_ms + ' ms   ' +
    s.vision.det_count + ' detection(s)   frames ' + s.vision.frames +
    (s.vision.calibrated ? '' : '   [NO CALIBRATION MATRIX]') +
    (s.vision.error ? '   [' + s.vision.error + ']' : '');

  document.getElementById('link-note').textContent =
    s.serial.connected
      ? (s.serial.port + ' - Mega answering' + (radio && radio.detail ? '  |  ' + radio.detail : ''))
    : s.serial.state === 'verifying'
      ? (s.serial.port + ' - opened, waiting for the Mega to answer...')
    : (s.serial.error || (radio && radio.error) || (s.serial.port + ' - ' + s.serial.state));

  const m = s.mega, p = s.picks;
  let h = '';
  h += row('mode', s.mode + (s.switching ? ' (switching)' : ''));
  h += row('auto state', s.auto_state);
  h += row('pick in progress', s.pick_busy ? 'YES' : 'no');
  h += row('platform', s.driving ? 'DRIVING FORWARD' : (m.drive && m.drive !== '0' ? m.drive : 'stopped'));
  h += row('bluetooth radio', !radio ? 'n/a'
        : (radio.connected ? 'linked' : (radio.state === 'connecting' ? 'connecting...' : 'DOWN')));
  h += row('mega link', s.serial.state + (s.mega_fresh ? '' : ' (no status)'));
  if(s.mega_fresh){
    h += row('homed', m.homed ? 'YES' : 'NO');
    h += row('steppers', m.drv ? 'on' : 'off');
    h += row('servo', m.att ? 'attached (' + m.grip + ')' : 'detached');
    h += row('position', 'X' + m.x + '  Y' + m.y + '  Z' + m.z);
    h += row('drive speed', m.spd);
    h += row('firmware busy', m.busy ? 'YES' : 'no');
  }
  h += row('last detection', d ? (d.label + ' ' + d.conf.toFixed(2) +
        ' @ ' + d.px + ',' + d.py + 'px' +
        (d.x_mm === null ? '' : ' = X' + d.x_mm + ' Y' + d.y_mm + ' (r' + d.r_mm + ')') +
        '  ' + d.age + 's ago') : '-');
  h += row('picks', p.ok + ' ok / ' + p.err + ' failed');
  if(p.last) h += row('last pick', 'X' + p.last.x.toFixed(1) + ' Y' +
        p.last.y.toFixed(1) + ' -> ' + p.last.result);
  document.getElementById('kv').innerHTML = h;
}

async function pollLog(){
  try{
    const j = await (await fetch('/api/log?since=' + logSeq)).json();
    logSeq = j.seq;
    if(!j.lines.length) return;
    const c = document.getElementById('console');
    const stick = c.scrollTop + c.clientHeight >= c.scrollHeight - 30;
    for(const [seq, ts, src, text] of j.lines){
      const div = document.createElement('div');
      const bad = /^(ERR|!!|\*\*)/.test(text);
      div.className = 's-' + (bad ? 'err' : src);
      const t = new Date(ts * 1000).toLocaleTimeString('en-GB');
      div.textContent = t + '  ' + src.padEnd(4) + '  ' + text;
      c.appendChild(div);
    }
    while(c.childElementCount > 400) c.removeChild(c.firstChild);
    if(stick) c.scrollTop = c.scrollHeight;
  }catch(e){ /* transient */ }
}

poll(); pollLog();
setInterval(poll, 450);
setInterval(pollLog, 700);
</script>
</body>
</html>
"""


# ======================================================================
#  FLASK APP
# ======================================================================
app = Flask(__name__)
VISION = None
LINK = None
CTL = None
ARGS = None
RFCOMM = None


def _json():
    return request.get_json(silent=True) or {}


def _reply(ok, error="", **extra):
    payload = {"ok": bool(ok)}
    if error:
        payload["error"] = error
    payload.update(extra)
    return jsonify(payload), (200 if ok else 409)


@app.after_request
def _nocache(resp):
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.route("/")
def index():
    return Response(PAGE, mimetype="text/html")


@app.route("/stream")
def stream():
    return Response(VISION.frames_iter(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/snapshot.jpg")
def snapshot():
    data = VISION.snapshot()
    if not data:
        return Response("no frame yet", status=503)
    return Response(data, mimetype="image/jpeg")


@app.route("/favicon.ico")
def favicon():
    # Browsers ask for this unprompted; answering keeps the console clean.
    return Response(b"", mimetype="image/x-icon")


@app.route("/snapshot_raw.jpg")
def snapshot_raw():
    """The newest frame with NO overlays, for calibration.

    calibrate.py cannot reliably open this camera itself - the node wedges
    and every open blocks for ~50 s. demo.py retries in the background and
    does eventually get through, so once the stream is live you can pull a
    clean frame from here instead of fighting for the device:
        python calibrate.py capture      (falls back to this automatically)
    """
    data = VISION.raw_jpeg
    if not data:
        return Response("no frame yet", status=503)
    age = time.time() - VISION.raw_ts
    resp = Response(data, mimetype="image/jpeg")
    resp.headers["X-Frame-Age"] = "%.2f" % age
    return resp


@app.route("/api/status")
def api_status():
    return jsonify(CTL.status())


@app.route("/api/log")
def api_log():
    try:
        since = int(request.args.get("since", 0))
    except ValueError:
        since = 0
    rows, seq = LOG.since(since)
    # Cap the payload, but report the seq of what was actually sent -
    # reporting the newest seq here would silently drop the overflow.
    if len(rows) > 300:
        rows = rows[-300:]
        seq = rows[-1][0]
    return jsonify({"lines": rows, "seq": seq})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    ok, why = CTL.set_mode(_json().get("mode", ""))
    return _reply(ok, why)


@app.route("/api/estop", methods=["POST"])
def api_estop():
    ok, why = CTL.estop()
    return _reply(ok, why)


@app.route("/api/settings", methods=["POST"])
def api_settings():
    applied = SET.update(_json())
    return _reply(True, applied=applied)


# ----------------------------------------------------------------------
#  Manual endpoints. Every one of them goes through Controller.manual_send,
#  so the AUTO / pick-busy gate is applied in exactly one place.
# ----------------------------------------------------------------------
ARM_ACTIONS = {
    "enable":        "E",
    "release":       "D",
    "steppers_on":   "ES",
    "steppers_off":  "DS",
    "servo_attach":  "GATT",
    "servo_detach":  "GDET",
    "sethome":       "SETHOME",
    "unhome":        "UNHOME",
    "home":          "HOME",
    "work":          "WORK",
    "bin":           "BIN",
    "park":          "PARK",
    "pos":           "POS",
    "limits":        "LIMITS",
    "grip_open":     "GO",
    "grip_close":    "GC",
    "grip_plus":     "GP",
    "grip_minus":    "GM",
}


@app.route("/api/arm", methods=["POST"])
def api_arm():
    action = _json().get("action", "")
    cmd = ARM_ACTIONS.get(action)
    if cmd is None:
        return _reply(False, "unknown arm action: %s" % action)
    ok, why, _ = CTL.manual_send(cmd)
    return _reply(ok, why, sent=cmd)


@app.route("/api/jog", methods=["POST"])
def api_jog():
    d = _json()
    try:
        delta = float(d.get("delta", 0))
    except (TypeError, ValueError):
        return _reply(False, "bad delta")
    if delta == 0:
        return _reply(False, "zero jog")
    ok, why, reply = CTL.jog(d.get("axis", ""), delta)
    return _reply(ok, why, reply=reply)


DRIVE_CMDS = {"F", "B", "L", "R", "T", "X", "0"}


@app.route("/api/drive", methods=["POST"])
def api_drive():
    cmd = str(_json().get("cmd", "")).upper()
    if cmd not in DRIVE_CMDS:
        return _reply(False, "unknown drive command")
    if cmd == "0":
        # Stopping is never gated - it is always safe and always allowed.
        LINK.interrupt("0")
        return _reply(True)
    ok, why, _ = CTL.manual_send(cmd)
    return _reply(ok, why)


@app.route("/api/pick", methods=["POST"])
def api_pick():
    """One-shot manual pick. Uses the same busy flag as the auto loop, so
    the UI locks up exactly as it does during an automatic cycle."""
    d = _json()
    try:
        x, y = float(d["x"]), float(d["y"])
    except (KeyError, TypeError, ValueError):
        return _reply(False, "need numeric x and y (mm)")
    r = (x * x + y * y) ** 0.5
    if r > SET.safe_r:
        return _reply(False, "X%.1f Y%.1f is r=%.1fmm, outside the %.0fmm reach"
                             % (x, y, r, SET.safe_r))
    if x >= SET.keep_x:
        return _reply(False, "X%.1f is inside the bin keep-out (X>=%.0f) - the "
                             "arm would answer ERR TARGET" % (x, SET.keep_x))
    ok, why = CTL.manual_gate()
    if not ok:
        return _reply(False, why)

    def run():
        CTL.set_pick_busy(True)
        try:
            line = LINK.command("PICK X%.1f Y%.1f" % (x, y),
                                lambda src, t: t == "DONE" or t.startswith("ERR"),
                                SET.pick_timeout_s)
            CTL.auto.last_pick = {"x": x, "y": y,
                                  "result": line or "TIMEOUT", "ts": time.time()}
            if line == "DONE":
                CTL.auto.picks_ok += 1
            else:
                CTL.auto.picks_err += 1
        finally:
            CTL.set_pick_busy(False)

    threading.Thread(target=run, daemon=True).start()
    return _reply(True)


@app.route("/api/raw", methods=["POST"])
def api_raw():
    line = str(_json().get("line", "")).strip()
    if not line:
        return _reply(False, "empty command")
    if line in ("0", "STOP", "stop"):
        CTL.estop()
        return _reply(True)
    ok, why, _ = CTL.manual_send(line)
    return _reply(ok, why)


@app.route("/api/serial", methods=["POST"])
def api_serial():
    action = _json().get("action", "")
    if action == "connect":
        LINK.connect_async()
        return _reply(True)
    if action == "disconnect":
        if CTL.mode == AUTO or CTL.pick_busy:
            return _reply(False, "stop autoweeding before dropping the link")
        LINK.disconnect()
        return _reply(True)
    if action in ("rebind", "reconnect"):
        if CTL.mode == AUTO or CTL.pick_busy:
            return _reply(False, "stop autoweeding before reconnecting")
        LINK.disconnect()
        RFCOMM.stop()
        time.sleep(0.5)
        ok = RFCOMM.start()
        LINK.want_connected = True
        LINK.connect_async()
        return _reply(True, radio_up=ok, detail=RFCOMM.last_error)
    return _reply(False, "unknown serial action")


# ======================================================================
#  ENTRY POINT
# ======================================================================
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Weed robot: detection stream + web control panel",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--model", default=MODEL_PATH)
    p.add_argument("--cam", type=int, default=CAM_INDEX)
    p.add_argument("--imgsz", type=int, default=IMG_SIZE)
    p.add_argument("--conf", type=float, default=CONF_THRESH)
    p.add_argument("--width", type=int, default=CAM_WIDTH)
    p.add_argument("--height", type=int, default=CAM_HEIGHT)
    p.add_argument("--quality", type=int, default=JPEG_QUALITY)
    p.add_argument("--calib", default=CALIBRATION_FILE)
    p.add_argument("--bt-port", default=BT_PORT)
    p.add_argument("--bt-mac", default=BT_MAC)
    p.add_argument("--bt-chan", type=int, default=BT_CHAN)
    p.add_argument("--baud", type=int, default=BAUD_RATE)
    p.add_argument("--rfcomm-dev", type=int, default=0,
                   help="the N in /dev/rfcommN")
    p.add_argument("--no-bind", "--no-rfcomm", action="store_true",
                   dest="no_bind",
                   help="do not raise the Bluetooth link; assume something "
                        "else already connected /dev/rfcommN")
    p.add_argument("--no-serial", action="store_true",
                   help="run the UI with no Mega at all (stream/dev only)")
    p.add_argument("--safe-r", type=float, default=SAFE_R_MM,
                   help="max pick radius in mm; mirror the firmware SAFE_R")
    p.add_argument("--keep-x", type=float, default=KEEP_X_MM,
                   help="bin keep-out: weeds at X >= this are never picked "
                        "(re-synced from the Mega's STATUS kpx)")
    p.add_argument("--start-auto", action="store_true",
                   help="begin in autoweeding mode instead of manual")
    p.add_argument("--port", type=int, default=HTTP_PORT)
    p.add_argument("--host", default="0.0.0.0")
    return p.parse_args(argv)


def main():
    global VISION, LINK, CTL, ARGS, RFCOMM
    ARGS = parse_args()

    SET.conf_thresh = ARGS.conf
    SET.safe_r = ARGS.safe_r
    SET.keep_x = ARGS.keep_x

    _quieten_werkzeug()
    LOG.add("app", "demo.py starting")

    # 1. Raise the Bluetooth link, so you never type an rfcomm command
    #    by hand again. This is a connect, not a bind - see the Rfcomm
    #    class for why bind does not actually work here.
    RFCOMM = Rfcomm(ARGS.rfcomm_dev, ARGS.bt_mac, ARGS.bt_chan, ARGS.bt_port,
                    enabled=not (ARGS.no_serial or ARGS.no_bind))
    if RFCOMM.enabled:
        RFCOMM.start()

    # 2. Vision comes up on its own thread - the model takes a while to
    #    load and the web UI should be reachable long before it finishes.
    VISION = Vision(ARGS.model, ARGS.cam, ARGS.imgsz, ARGS.width, ARGS.height,
                    ARGS.quality, ARGS.calib)
    VISION.start()

    # 3. Serial + arbitration.
    LINK = SerialLink(ARGS.bt_port, ARGS.baud, rfcomm=RFCOMM)
    CTL = Controller(LINK, VISION)
    VISION.status_provider = CTL.overlay_status

    if not ARGS.no_serial:
        LINK.connect_async()
        threading.Thread(target=housekeeping, args=(LINK, CTL), daemon=True).start()

    if ARGS.start_auto:
        def arm_later():
            for _ in range(40):
                if LINK.connected:
                    CTL.set_mode(AUTO)
                    return
                time.sleep(0.5)
            LOG.add("app", "--start-auto: Mega never connected, staying manual")
        threading.Thread(target=arm_later, daemon=True).start()

    ip = local_ip()
    LOG.add("app", "control panel: http://%s:%d" % (ip, ARGS.port))
    print("\n  ==> open  http://%s:%d  in a browser on the same wifi\n"
          % (ip, ARGS.port), flush=True)

    try:
        app.run(host=ARGS.host, port=ARGS.port, threaded=True,
                debug=False, use_reloader=False)
    finally:
        try:
            CTL.auto.request_stop()
            LINK.interrupt("0")
        except Exception:
            pass
        VISION.stop()
        LINK.disconnect()
        # The link only lives as long as our rfcomm child, so hand it back
        # rather than leaving a half-owned device node behind.
        RFCOMM.stop()


if __name__ == "__main__":
    main()
