#!/usr/bin/env python3
"""
UART TX diagnostic for the Raspberry Pi <-> DWIN link.

Symptom this isolates: touch frames reach the Pi (RX works) but writes never
reach the DWIN (page doesn't switch, buzzer silent). That means the Pi -> DWIN
TX direction is broken. This script finds WHERE the break is.

----------------------------------------------------------------------------
STEP 1 - prove the Pi itself transmits (loopback). Run this FIRST.
----------------------------------------------------------------------------
  * Power off / unplug the DWIN.
  * Jumper Pi GPIO14 (TXD, pin 8) directly to GPIO15 (RXD, pin 10).
    If a level shifter is in the way, bypass it for this test (jumper the
    bare Pi pins).
  * Run:  python3 uart_loopback_test.py
  * PASS  ("loopback OK")  -> the Pi UART transmits fine. The fault is on the
            wire: level-shifter TX channel, a loose/swapped TX line, the DWIN
            RX pin, or the DWIN rejecting frames (CRC enabled). Go to STEP 2.
  * FAIL  ("loopback FAILED") -> the Pi is NOT transmitting. Fix the Pi UART
            config (hints printed on failure); the DWIN side is irrelevant.

----------------------------------------------------------------------------
STEP 2 - if loopback PASSED, reconnect the DWIN and fire raw frames at it:
----------------------------------------------------------------------------
  * Run:  python3 uart_loopback_test.py --dwin
  * A reply to the read  -> the DWIN receives + answers: TX works end-to-end,
            so a page not switching is a DGUS project issue (page id / page
            empty), NOT comms.
  * No reply at all      -> DWIN not accepting frames: CRC enabled in the DGUS
            config, wrong baud, or TX wire not landing on the DWIN RX pin.
"""

import argparse
import time

import serial


def loopback(port: str, baud: int) -> None:
    ser = serial.Serial(port, baud, timeout=0.3,
                        bytesize=8, parity="N", stopbits=1)
    ser.reset_input_buffer()
    payload = bytes([0x5A, 0xA5, 0x01, 0x02, 0x03, 0x04, 0x05])
    print("TX:", payload.hex(" "))
    ser.write(payload)
    ser.flush()
    time.sleep(0.1)
    rx = ser.read(len(payload))
    ser.close()
    print("RX:", rx.hex(" ") if rx else "(nothing)")
    if rx == payload:
        print("\nloopback OK -> the Pi UART transmits. The break is on the wire:"
              "\n  level-shifter TX channel / loose-swapped TX line / DWIN RX pin"
              "\n  / DWIN CRC enabled. Reconnect DWIN and run with --dwin.")
    else:
        print("\nloopback FAILED -> the Pi is not transmitting (or wrong port/baud)."
              "\n  - ls -l /dev/serial0        (-> ttyAMA0 good; ttyS0 = mini-UART)"
              "\n  - /boot/firmware/config.txt : enable_uart=1   and   dtoverlay=disable-bt"
              "\n  - /boot/firmware/cmdline.txt: remove  console=serial0,115200"
              "\n  - is the GPIO14<->GPIO15 jumper actually on?"
              "\n  (older Pi OS: files live in /boot/ not /boot/firmware/)")


def dwin_raw(port: str, baud: int) -> None:
    ser = serial.Serial(port, baud, timeout=0.4,
                        bytesize=8, parity="N", stopbits=1)
    frames = [
        ("read current page (VP 0x0014)",
         bytes([0x5A, 0xA5, 0x04, 0x83, 0x00, 0x14, 0x01])),
        ("page switch -> page 2",
         bytes([0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x02])),
        ("buzzer 250 ms",
         bytes([0x5A, 0xA5, 0x05, 0x82, 0x00, 0xA0, 0x00, 0x20])),
    ]
    for label, frame in frames:
        ser.reset_input_buffer()
        print(f"\n{label}\n  TX: {frame.hex(' ')}")
        ser.write(frame)
        ser.flush()
        time.sleep(0.2)
        reply = ser.read(32)
        print("  reply:", reply.hex(" ") if reply else "(nothing)")
    ser.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Pi<->DWIN UART TX diagnostic")
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dwin", action="store_true",
                    help="STEP 2: fire raw read/page/beep frames at a connected DWIN")
    args = ap.parse_args()
    if args.dwin:
        dwin_raw(args.port, args.baud)
    else:
        loopback(args.port, args.baud)


if __name__ == "__main__":
    main()
