#!/usr/bin/env python3
"""
EZVIZ H8c  pan/tilt control + motion/human-detection read-out, via the EZVIZ
CLOUD API (community library `pyezvizapi`).

IMPORTANT — read this before expecting it to work:
  * PTZ and AI/human-detection on EZVIZ are NOT exposed on the local network.
    They go through your EZVIZ ACCOUNT in the cloud, so this script needs:
      - internet access on the PC,
      - your EZVIZ account email + password,
      - the camera SERIAL (on the label / in the app; e.g. "BB1234567").
  * The API is reverse-engineered (no official docs); method names can change
    between library versions. If a call below errors, run `pyezvizapi --help`
    and `python -c "import pyezvizapi,inspect;print(dir(pyezvizapi))"` to see
    what your installed version exposes, and adjust.

Install:
    pip install pyezvizapi

Credentials via env vars (don't hard-code):
    PowerShell:
        $env:EZVIZ_USER="you@email.com"
        $env:EZVIZ_PASS="your_account_password"
        $env:EZVIZ_SERIAL="BB1234567"

Run:
    python ptz_and_detection.py status            # print status incl. motion
    python ptz_and_detection.py move up 5          # pan/tilt: dir + speed(1-7)
    python ptz_and_detection.py move left 3
"""

import os
import sys

# region: "eu" / "us" / or full host like "apiieu.ezvizlife.com".
REGION = os.environ.get("EZVIZ_REGION", "eu")


def get_client():
    try:
        from pyezvizapi.client import EzvizClient
    except ImportError:
        print("[error] pyezvizapi not installed. Run: pip install pyezvizapi",
              file=sys.stderr)
        sys.exit(1)

    # Fall back to interactive prompts if env vars are not set.
    user = os.environ.get("EZVIZ_USER") or input("EZVIZ account email: ").strip()
    pwd = os.environ.get("EZVIZ_PASS")
    if not pwd:
        import getpass
        pwd = getpass.getpass("EZVIZ account password: ")
    if not user or not pwd:
        print("[error] EZVIZ account email + password required.", file=sys.stderr)
        sys.exit(1)

    client = EzvizClient(user, pwd, REGION)
    client.login()
    return client


def cmd_status(client, serial):
    """Dump everything the cloud knows, then highlight detection fields."""
    info = client.get_device_infos(serial)
    print("=== raw device info ===")
    import json
    print(json.dumps(info, indent=2, default=str))

    # Detection-related fields seen across EZVIZ models (names vary by firmware):
    print("\n=== detection-related ===")
    for key in ("Motion_Trigger", "alarm_notify", "PIR_Status",
                "human_detect", "intelligent_detection"):
        # info may be nested; do a shallow search.
        val = _find_key(info, key)
        print(f"  {key:22}: {val}")


def cmd_move(client, serial, direction, speed):
    """Nudge the camera: START moving, wait briefly, then STOP.

    pyezvizapi 1.0.x signature:
        ptz_control(command, serial, action, speed=5)
        command = "UP"|"DOWN"|"LEFT"|"RIGHT"   action = "START"|"STOP"
    """
    import time
    command = direction.upper()
    valid = ("UP", "DOWN", "LEFT", "RIGHT")
    if command not in valid:
        print(f"[error] direction must be one of {valid}", file=sys.stderr)
        sys.exit(1)

    speed = max(1, min(int(speed), 7))
    client.ptz_control(command, serial, "START", speed)
    time.sleep(0.6)                       # how long to move; tune for a bigger/smaller step
    client.ptz_control(command, serial, "STOP", speed)
    print(f"PTZ {command} at speed {speed} (0.6s nudge)")


def _find_key(obj, key):
    """Recursively search a nested dict/list for the first matching key."""
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for v in obj.values():
            r = _find_key(v, key)
            if r is not None:
                return r
    elif isinstance(obj, list):
        for v in obj:
            r = _find_key(v, key)
            if r is not None:
                return r
    return None


def main():
    # 1) Parse the command FIRST, so `--help`/no-args never prompts for creds.
    args = sys.argv[1:]
    if not args or args[0] not in ("status", "move"):
        print(__doc__)
        sys.exit(0)
    if args[0] == "move" and len(args) < 2:
        print("[error] usage: move <up|down|left|right> [speed 1-7]", file=sys.stderr)
        sys.exit(1)

    # 2) Now resolve the camera serial.
    serial = os.environ.get("EZVIZ_SERIAL")
    if not serial:
        serial = input("Camera serial (on label / in app, e.g. BB1234567): ").strip()
    if not serial:
        print("[error] camera serial required.", file=sys.stderr)
        sys.exit(1)

    # 3) Log in to the EZVIZ cloud.
    client = get_client()
    try:
        if args[0] == "status":
            cmd_status(client, serial)
        elif args[0] == "move" and len(args) >= 2:
            direction = args[1]
            speed = args[2] if len(args) >= 3 else "3"
            cmd_move(client, serial, direction, speed)
        else:
            print(__doc__)
    finally:
        # be polite to the cloud session
        close = getattr(client, "close_session", None) or getattr(client, "logout", None)
        if close:
            try:
                close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
