#!/usr/bin/env python3
"""
btcheck.py - is the Mega actually talking, and if not, which half is broken?

"No reply from the Mega" has three quite different causes and they need
three different fixes, so this separates them:

  1. no Bluetooth link          -> HC-05 unpowered / out of range / unpaired
  2. link up, Mega never speaks  -> Mega not running, or TX1(D18) -> HC-05
                                    RXD broken
  3. link up, Mega speaks only
     after you press RESET       -> Mega is alive and its TX works, but it
                                    is not HEARING us: RX1(D19) <- HC-05
                                    TXD is the broken wire

Run it on the Pi, in the venv:

    cd ~/fyp
    sudo ./venv/bin/python btcheck.py            # probe test (~25 s)
    sudo ./venv/bin/python btcheck.py --reset    # then press the Mega's
                                                 # RESET button when told

sudo is needed for rfcomm; --reset is the decisive one, because the
firmware prints its banner on boot to Bluetooth as well as USB.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing - run this with ~/fyp/venv/bin/python")

MAC = os.environ.get("BT_MAC", "00:00:00:00:00:00")  # your HC-05 - see README
DEV = 0
CHAN = 1
PORT = "/dev/rfcomm0"
BAUD = 9600


def sudo(*args):
    base = [] if os.geteuid() == 0 else ["sudo", "-n"]
    return base + ["rfcomm"] + list(args)


def link_line():
    try:
        r = subprocess.run(["rfcomm", "-a"], capture_output=True, text=True,
                           timeout=8)
    except Exception:
        return ""
    for line in (r.stdout or "").splitlines():
        if line.strip().startswith("rfcomm%d:" % DEV):
            return line.strip()
    return ""


def connected():
    return " connected" in link_line()


def raise_link(timeout=25):
    if connected():
        print("[bt] already connected: %s" % link_line())
        return None
    if shutil.which("rfcomm") is None:
        sys.exit("rfcomm not installed - sudo apt install bluez")
    for attempt in (1, 2):
        subprocess.run(sudo("release", str(DEV)), capture_output=True,
                       text=True, timeout=10)
        if attempt == 2:
            time.sleep(2.0)
        print("[bt] rfcomm connect %d %s ch%d (attempt %d)..."
              % (DEV, MAC, CHAN, attempt))
        out = open("/tmp/btcheck_rfcomm.log", "w+")
        proc = subprocess.Popen(sudo("connect", str(DEV), MAC, str(CHAN)),
                                stdout=out, stderr=subprocess.STDOUT)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if proc.poll() is not None:
                out.seek(0)
                why = out.read().strip().replace("\n", "; ")
                print("[bt] connect failed: %s" % why)
                break
            if connected():
                print("[bt] LINK UP: %s" % link_line())
                return proc
            time.sleep(0.4)
        else:
            proc.terminate()
            print("[bt] timed out waiting for the link")
    return False


def listen(ser, seconds, probes=(), label=""):
    """Read for `seconds`, sending each probe at 4-second intervals."""
    got = bytearray()
    t0 = time.time()
    next_probe = 0
    i = 0
    while time.time() - t0 < seconds:
        if probes and time.time() - t0 >= next_probe:
            p = probes[i % len(probes)]
            i += 1
            next_probe += 4
            ser.write(p.encode() + b"\n")
            print("    -> sent %r" % p)
        chunk = ser.read(256)
        if chunk:
            got += chunk
            print("    <- %r" % chunk[:160])
    return bytes(got)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reset", action="store_true",
                    help="watch for the boot banner while you press RESET")
    ap.add_argument("--seconds", type=float, default=25.0)
    args = ap.parse_args()

    proc = raise_link()
    if proc is False:
        print("\nVERDICT: no Bluetooth link at all.")
        print("  The HC-05 is not reachable. Check it has power (its LED "
              "should be blinking) and that the Pi is still paired:")
        print("    bluetoothctl info %s" % MAC)
        return 1

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except Exception as e:
        print("\nVERDICT: link is up but %s would not open: %s" % (PORT, e))
        return 1

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    if args.reset:
        print("\n>>> PRESS THE RESET BUTTON ON THE MEGA NOW <<<")
        print("    (watching for %.0f seconds - the firmware prints its "
              "banner on boot)" % args.seconds)
        data = listen(ser, args.seconds)
        print()
        if data:
            print("VERDICT: the Mega IS alive and its TX path to the HC-05 "
                  "WORKS.")
            print("  It printed its boot banner over Bluetooth, so D18/TX1 -> "
                  "HC-05 RXD is fine.")
            print("  If it still ignores commands, the broken direction is the "
                  "other one:")
            print("    HC-05 TXD -> Mega D19/RX1.  Check that wire and that "
                  "both share a ground.")
        else:
            print("VERDICT: nothing at all on reset.")
            print("  The Mega is not transmitting. Either the sketch is not "
                  "running (check the Mega's ON led, and whether it boots on "
                  "USB), or D18/TX1 -> HC-05 RXD is broken.")
            print("  Note the HC-05 answering Bluetooth proves only that the "
                  "MODULE has power - not the board.")
        return 0

    print("\n[probe] sending STATUS / POS / HELP for %.0f s ..." % args.seconds)
    data = listen(ser, args.seconds, probes=("STATUS", "POS", "HELP"))
    print()
    if data:
        print("VERDICT: the Mega is answering. %d bytes received." % len(data))
        if b"ST " in data:
            print("  It replied to STATUS, so the updated firmware is flashed.")
        elif b"? STATUS" in data:
            print("  It replied '? STATUS', so the OLD firmware is still on "
                  "the board - flash the updated Robot_Master.ino to get the "
                  "STATUS and JOGX/Y/Z commands demo.py uses.")
        return 0

    print("VERDICT: Bluetooth is connected but the Mega said nothing in %.0f s."
          % args.seconds)
    print("  The HC-05 radio is fine - that is all a working Bluetooth link "
          "proves. Now find out whether the BOARD is running:")
    print("    sudo ./venv/bin/python btcheck.py --reset")
    print("  and press the Mega's RESET button when it tells you to.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
