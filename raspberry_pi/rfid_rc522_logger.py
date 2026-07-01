#!/usr/bin/env python3
"""
CrowPi 2 / Raspberry Pi 4 - RFID-RC522 tag reader and CSV logger.

Continuously polls the RC522 reader (on SPI bus 0). Each time a NEW tag is
tapped, it prints the tag's UID + any stored text and appends one row to
rfid_log.csv. Holding the same tag down does not create duplicate rows;
removing and re-tapping the same tag logs a fresh row.

Setup (see the notes the assistant printed, or the README):
    1. Enable SPI:   sudo raspi-config -> Interface Options -> SPI -> Yes -> reboot
    2. Install lib:  pip3 install mfrc522
       (mfrc522 pulls in spidev and RPi.GPIO automatically)

Run:
    python3 rfid_rc522_logger.py
"""

import csv
import os
import time
from datetime import datetime

import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

# CSV lives next to this script, regardless of the current working directory.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, "rfid_log.csv")
CSV_HEADER = ["timestamp", "uid_dec", "uid_hex", "text"]

POLL_DELAY = 0.1   # seconds between reads while polling
MISS_RESET = 3     # consecutive empty polls before a tag counts as "removed"


def ensure_csv(path):
    """Write the header row once, when the file is first created."""
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        with open(path, "w", newline="") as f:
            csv.writer(f).writerow(CSV_HEADER)


def append_row(path, row):
    """Append a single tag scan as a new CSV row, flushed to disk immediately."""
    with open(path, "a", newline="") as f:
        csv.writer(f).writerow(row)
        f.flush()


def main():
    reader = SimpleMFRC522()
    ensure_csv(CSV_FILE)
    print(f"RC522 ready - tap a tag on the reader.  (Ctrl+C to quit)")
    print(f"Logging to: {CSV_FILE}\n")

    last_uid = None   # UID of the tag currently on the reader (None = field empty)
    missed = 0        # count of consecutive empty polls (debounces read glitches)

    try:
        while True:
            uid, text = reader.read_no_block()

            if uid is None:
                # No tag in the field. Require a few empty polls in a row before
                # declaring the tag gone, so a momentary read glitch does not
                # cause the same held tag to be logged twice.
                missed += 1
                if missed >= MISS_RESET:
                    last_uid = None
                time.sleep(POLL_DELAY)
                continue

            missed = 0

            if uid == last_uid:
                # Same tag still held down - already logged, skip.
                time.sleep(POLL_DELAY)
                continue

            # New tag detected -> log exactly one row.
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            uid_hex = format(uid, "X")
            text = (text or "").strip()
            row = [timestamp, uid, uid_hex, text]

            append_row(CSV_FILE, row)
            print(f"[{timestamp}]  UID={uid}  (0x{uid_hex})  text={text!r}")

            last_uid = uid

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
