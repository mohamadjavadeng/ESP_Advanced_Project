#!/usr/bin/env python3
"""
EZVIZ H8c  pan/tilt (PTZ) control from Python.

PTZ on EZVIZ is NOT available on the local network — it goes through your EZVIZ
ACCOUNT in the cloud. So this needs: internet, EZVIZ app email + password, and the
camera serial (on the label / in the app, e.g. "BB1234567").

Install once:
    pip install pyezvizapi

Credentials (set as env vars to avoid prompts):
    $env:EZVIZ_USER="you@email.com"
    $env:EZVIZ_PASS="account_password"
    $env:EZVIZ_SERIAL="BB1234567"
    $env:EZVIZ_REGION="eu"          # eu / us  (default eu)

Usage:
    python ptz.py arrows                 # interactive: drive with arrow keys
    python ptz.py move up 5              # one nudge: dir + speed(1-7)
    python ptz.py goto 0.4 0.6          # set angle: x,y in 0.0..1.0 (pan,tilt)

In 'arrows' mode:
    Arrow keys = pan/tilt        + / - = change speed
    q or Esc   = quit
"""

import os
import sys
import time

DIRECTIONS = ("UP", "DOWN", "LEFT", "RIGHT")
REGION = os.environ.get("EZVIZ_REGION", "eu")
NUDGE_SEC = 0.5          # how far one key press / nudge moves the camera


# --------------------------------------------------------------- cloud login --
def get_client():
    try:
        from pyezvizapi.client import EzvizClient
    except ImportError:
        sys.exit("[error] pyezvizapi not installed. Run: pip install pyezvizapi")

    user = os.environ.get("EZVIZ_USER") or input("EZVIZ account email: ").strip()
    pwd = os.environ.get("EZVIZ_PASS")
    if not pwd:
        import getpass
        pwd = getpass.getpass("EZVIZ account password: ")
    if not user or not pwd:
        sys.exit("[error] account email + password required.")

    client = EzvizClient(user, pwd, REGION)
    print("Logging in to EZVIZ cloud...")
    client.login()
    return client


def get_serial():
    serial = os.environ.get("EZVIZ_SERIAL")
    if not serial:
        serial = input("Camera serial (label / app, e.g. BB1234567): ").strip()
    if not serial:
        sys.exit("[error] camera serial required.")
    return serial


# --------------------------------------------------------------------- moves --
def nudge(client, serial, direction, speed, duration=NUDGE_SEC):
    """START moving, wait, STOP. pyezvizapi: ptz_control(command, serial, action, speed)."""
    command = direction.upper()
    if command not in DIRECTIONS:
        print(f"[error] direction must be one of {DIRECTIONS}")
        return
    speed = max(1, min(int(speed), 7))
    client.ptz_control(command, serial, "START", speed)
    time.sleep(duration)
    client.ptz_control(command, serial, "STOP", speed)
    print(f"  {command} @ speed {speed}")


def goto(client, serial, x, y):
    """Move to an absolute angle. x = pan, y = tilt, both 0.0..1.0."""
    x = max(0.0, min(float(x), 1.0))
    y = max(0.0, min(float(y), 1.0))
    client.ptz_control_coordinates(serial, x, y)
    print(f"moved to coordinates pan={x:.2f} tilt={y:.2f}")


# ---------------------------------------------------------- arrow-key driver --
def read_key():
    """Blocking single-keypress read on Windows; returns a simple token string."""
    import msvcrt
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):          # arrow / function key -> second byte
        code = msvcrt.getch()
        return {b"H": "UP", b"P": "DOWN", b"K": "LEFT", b"M": "RIGHT"}.get(code, "")
    if ch in (b"q", b"Q", b"\x1b"):       # q or Esc
        return "QUIT"
    if ch in (b"+", b"="):
        return "FASTER"
    if ch == b"-":
        return "SLOWER"
    return ""


def arrows(client, serial):
    speed = 4
    print("\nArrow keys = pan/tilt   |   +/- = speed   |   q/Esc = quit")
    print(f"speed = {speed}")
    while True:
        key = read_key()
        if key == "QUIT":
            print("done.")
            return
        elif key == "FASTER":
            speed = min(speed + 1, 7); print(f"speed = {speed}")
        elif key == "SLOWER":
            speed = max(speed - 1, 1); print(f"speed = {speed}")
        elif key in DIRECTIONS:
            nudge(client, serial, key, speed)


# ----------------------------------------------------------------------- main --
def main():
    args = sys.argv[1:]
    if not args or args[0] not in ("arrows", "move", "goto"):
        print(__doc__)
        sys.exit(0)

    cmd = args[0]
    if cmd == "move" and len(args) < 2:
        sys.exit("[error] usage: move <up|down|left|right> [speed 1-7]")
    if cmd == "goto" and len(args) < 3:
        sys.exit("[error] usage: goto <x 0..1> <y 0..1>")

    serial = get_serial()
    client = get_client()
    try:
        if cmd == "arrows":
            arrows(client, serial)
        elif cmd == "move":
            nudge(client, serial, args[1], args[2] if len(args) >= 3 else 4)
        elif cmd == "goto":
            goto(client, serial, args[1], args[2])
    finally:
        close = getattr(client, "close_session", None) or getattr(client, "logout", None)
        if close:
            try:
                close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
