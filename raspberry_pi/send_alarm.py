#!/usr/bin/env python3
"""
Quick test: send one alarm to the ESP32 server from your PC.

Requirements:
  * Your PC must be joined to the WiFi "ESP32_Server" (password 12345678).
  * pip install requests

Examples:
  python send_alarm.py                       # alarm ON, target 1.5 m
  python send_alarm.py --off                 # clear the alarm
  python send_alarm.py --target 2.0 --message "deep dig"
  python send_alarm.py --status              # just read /status
"""

import argparse
import sys

import requests

DEFAULT_URL = "http://192.168.4.1"


def main():
    ap = argparse.ArgumentParser(description="Send a test alarm to the ESP32 server")
    ap.add_argument("--url", default=DEFAULT_URL, help="server base URL")
    ap.add_argument("--target", type=float, default=1.5, help="target depth (m)")
    ap.add_argument("--message", default="PC test alarm", help="alarm message")
    ap.add_argument("--off", action="store_true", help="send alarm=false (clear)")
    ap.add_argument("--status", action="store_true",
                    help="only GET /status, do not send an alarm")
    args = ap.parse_args()

    try:
        if args.status:
            r = requests.get(f"{args.url}/status", timeout=5)
        else:
            payload = {
                "alarm": not args.off,
                "message": args.message,
                "target_depth": args.target,
            }
            print("POST /alarm ->", payload)
            r = requests.post(f"{args.url}/alarm", json=payload, timeout=5)

        r.raise_for_status()
        print(f"HTTP {r.status_code}")
        print(r.json())
    except requests.RequestException as e:
        print(f"[error] request failed: {e}", file=sys.stderr)
        print("Is your PC joined to the 'ESP32_Server' WiFi and the ESP32 powered?",
              file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
