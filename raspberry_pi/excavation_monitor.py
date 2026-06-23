#!/usr/bin/env python3
"""
Raspberry Pi 4 -> ESP32 Excavation Server monitor.

The ESP32 runs a SoftAP "ESP32_Server" at 192.168.4.1. Join that WiFi network
from the Pi either manually:

    sudo nmcli dev wifi connect "ESP32_Server" password "12345678"

or let this script do it (Raspberry Pi OS Bookworm / NetworkManager):

    python3 excavation_monitor.py --wifi-ssid ESP32_Server --wifi-password 12345678

WARNING: connecting to the ESP32 AP switches the Pi's WiFi, so if you are SSH'd
into the Pi over WiFi your session will drop. Use a wired/console link, or run
it on the Pi locally, when switching networks.

What this script does:
  * Sends the target excavation depth + an optional alarm/message to the server
    via  POST /alarm  (JSON).
  * Polls  GET /status  every --interval seconds, prints boom/stick angles and
    the computed depth, and logs each sample to a CSV file.
  * Raises a local alarm (console + optional GPIO buzzer) whenever the server
    reports depth_alarm or pi_alarm.

Dependencies:  pip3 install requests
Optional GPIO buzzer:  pip3 install RPi.GPIO   (set --buzzer-pin)
"""

import argparse
import csv
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

import requests

# --------------------------------------------------------------------------- #
DEFAULT_URL = "http://192.168.4.1"

# Optional GPIO buzzer. Stays a no-op unless --buzzer-pin is given and RPi.GPIO
# is installed (so the script also runs fine on a laptop for testing).
_GPIO = None
_BUZZER_PIN = None


def setup_buzzer(pin):
    global _GPIO, _BUZZER_PIN
    if pin is None:
        return
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        print("[warn] RPi.GPIO not available - buzzer disabled", file=sys.stderr)
        return
    _GPIO = GPIO
    _BUZZER_PIN = pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)


def buzzer(on):
    if _GPIO is not None and _BUZZER_PIN is not None:
        _GPIO.output(_BUZZER_PIN, _GPIO.HIGH if on else _GPIO.LOW)


def cleanup_buzzer():
    if _GPIO is not None and _BUZZER_PIN is not None:
        _GPIO.output(_BUZZER_PIN, _GPIO.LOW)
        _GPIO.cleanup(_BUZZER_PIN)


# --------------------------------------------------------------------------- #
def send_alarm(url, alarm, message, target_depth, timeout=5):
    """POST alarm + target depth to the server. Returns the server status dict."""
    payload = {"alarm": bool(alarm), "message": message,
               "target_depth": float(target_depth)}
    r = requests.post(f"{url}/alarm", json=payload, timeout=timeout)
    r.raise_for_status()
    return r.json()


def get_status(url, timeout=5):
    r = requests.get(f"{url}/status", timeout=timeout)
    r.raise_for_status()
    return r.json()


# --------------------------------------------------------------------------- #
# WiFi control (Raspberry Pi OS Bookworm = NetworkManager). Linux only; uses
# the `nmcli` CLI under the hood. On a non-NetworkManager system these become
# a graceful no-op so the script still runs for testing on a laptop.
def _nmcli(args, timeout=30):
    return subprocess.run(["nmcli", *args], capture_output=True, text=True,
                          timeout=timeout)


def current_ssid():
    """SSID of the currently-active WiFi connection, or None."""
    if shutil.which("nmcli") is None:
        return None
    try:
        r = _nmcli(["-t", "-f", "active,ssid", "dev", "wifi"])
    except (subprocess.SubprocessError, OSError):
        return None
    for line in r.stdout.splitlines():
        active, _, ssid = line.partition(":")   # e.g. "yes:ESP32_Server"
        if active == "yes":
            return ssid or None
    return None


def connect_wifi(ssid, password=None, timeout=45):
    """Connect the Pi to a WiFi network via NetworkManager.

    Returns True on success. Needs privileges: if it fails with an auth error,
    run the script with sudo, or store the connection once manually.
    """
    if shutil.which("nmcli") is None:
        print("[error] nmcli not found. WiFi control needs NetworkManager "
              "(Raspberry Pi OS Bookworm+). Connect manually on other systems.",
              file=sys.stderr)
        return False

    if current_ssid() == ssid:
        print(f"[wifi] already connected to '{ssid}'")
        return True

    try:
        _nmcli(["radio", "wifi", "on"])
        _nmcli(["dev", "wifi", "rescan"], timeout=20)
        time.sleep(2)                       # give the scan a moment to populate
        cmd = ["dev", "wifi", "connect", ssid]
        if password:
            cmd += ["password", password]
        print(f"[wifi] connecting to '{ssid}'...")
        r = _nmcli(cmd, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"[error] wifi connect to '{ssid}' timed out", file=sys.stderr)
        return False

    if r.returncode == 0:
        print(f"[wifi] connected to '{ssid}'")
        return True
    msg = (r.stderr or r.stdout).strip()
    print(f"[error] wifi connect failed: {msg}", file=sys.stderr)
    return False


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="ESP32 excavation server monitor")
    ap.add_argument("--url", default=DEFAULT_URL, help="server base URL")
    ap.add_argument("--target", type=float, default=1.5,
                    help="target excavation depth in metres")
    ap.add_argument("--message", default="monitor started",
                    help="message sent to the server with the target depth")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="status poll interval in seconds")
    ap.add_argument("--csv", default="excavation_log.csv",
                    help="CSV log file path")
    ap.add_argument("--buzzer-pin", type=int, default=None,
                    help="BCM pin for an alarm buzzer (optional)")
    ap.add_argument("--wifi-ssid", default=None,
                    help="connect to this WiFi via nmcli before monitoring "
                         "(e.g. ESP32_Server). WARNING: switches the Pi's WiFi, "
                         "so it will drop an SSH session running over WiFi.")
    ap.add_argument("--wifi-password", default=None,
                    help="password for --wifi-ssid")
    args = ap.parse_args()

    # Optionally join the target WiFi (e.g. the ESP32 SoftAP) first.
    if args.wifi_ssid:
        if not connect_wifi(args.wifi_ssid, args.wifi_password):
            sys.exit(1)
        time.sleep(2)                       # let DHCP assign an IP before HTTP

    setup_buzzer(args.buzzer_pin)

    running = {"on": True}

    def stop(_sig, _frm):
        running["on"] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    # Push the target depth to the server (retry until it answers).
    while running["on"]:
        try:
            send_alarm(args.url, False, args.message, args.target)
            print(f"[ok] target depth {args.target} m sent to {args.url}")
            break
        except requests.RequestException as e:
            print(f"[retry] cannot reach server ({e}); retrying in 2s...")
            time.sleep(2)

    # Open the CSV log.
    csv_file = open(args.csv, "a", newline="")
    writer = csv.writer(csv_file)
    if csv_file.tell() == 0:
        writer.writerow(["timestamp", "boom_deg", "stick_deg", "depth_m",
                         "target_depth", "depth_alarm", "pi_alarm"])

    print("Polling /status (Ctrl-C to stop)...")
    try:
        while running["on"]:
            try:
                s = get_status(args.url)
            except requests.RequestException as e:
                print(f"[warn] status request failed: {e}")
                time.sleep(args.interval)
                continue

            ts = datetime.now().isoformat(timespec="seconds")
            alarm = bool(s.get("depth_alarm")) or bool(s.get("pi_alarm"))
            buzzer(alarm)

            print(f"{ts}  boom={s.get('boom_deg'):.2f}  "
                  f"stick={s.get('stick_deg'):.2f}  "
                  f"depth={s.get('depth_m'):.3f}m / {s.get('target_depth'):.3f}m  "
                  f"{'*** ALARM ***' if alarm else ''}")

            writer.writerow([ts, s.get("boom_deg"), s.get("stick_deg"),
                             s.get("depth_m"), s.get("target_depth"),
                             s.get("depth_alarm"), s.get("pi_alarm")])
            csv_file.flush()
            time.sleep(args.interval)
    finally:
        buzzer(False)
        cleanup_buzzer()
        csv_file.close()
        print("\nstopped.")


if __name__ == "__main__":
    main()
