#!/usr/bin/env python3
"""
Event-driven DWIN example: handle TOUCH auto-upload frames and switch pages.

How the touch works (DGUS "touch -> variable data auto-upload"):
  * You touch a button on the panel -> the panel PUSHES a frame to the Pi:
        5A A5 06 83 <addrHi> <addrLo> 01 <valHi> <valLo>
    (it looks like a 0x83 read response; addr = the button's VP, value = data).
  * The Pi reacts, then writes 0x0000 back to that VP to "consume" the press so
    it does not retrigger.

This is the Python port of the Arduino readDWINFrame()/handleFrame()/
pageHandler() logic, using the exact HMI addresses:
    0x0200-0x020F  page buttons    (0x0200->page1, 0x0201->page2, ... )
    0x0220-0x022F  output buttons

Frame assembly is done inside the driver (dwin_lcd.read_event); here we just
dispatch on address, exactly like handleFrame().

Run from the raspberry_pi/ folder (so `import dwin_lcd` resolves):
    python3 dwin_page_example.py --port /dev/serial0 --baud 115200
"""

import argparse
import time

from dwin_lcd import DwinLCD, BuzzerDuration, DwinTimeout

# Page-button VP -> page number (exact HMI addresses, matching the Arduino code).
PAGE_BUTTONS = {
    0x0200: 1,
    0x0201: 2,
    0x0202: 3,
    0x0203: 4,
}

PAGE_RANGE = range(0x0200, 0x0210)     # 0x0200-0x020F -> pageHandler
OUTPUT_RANGE = range(0x0220, 0x0230)   # 0x0220-0x022F -> outputHandler
TOUCH_ON = 0x0001                      # value the button writes when pressed


class DwinUI:
    """Holds UI state and the per-address handlers (mirrors the Arduino code)."""

    def __init__(self, dwin: DwinLCD):
        self.dwin = dwin
        self.page_index = 0
        self.outputs = {}              # output VP -> bool (on/off)

    # handleFrame(): dispatch one received frame by address range.
    def handle_frame(self, ev):
        if ev.cmd != 0x83:             # ignore write ACKs / other frames
            return
        if ev.addr in PAGE_RANGE:
            self.page_handler(ev.addr, ev.value)
        elif ev.addr in OUTPUT_RANGE:
            self.output_handler(ev.addr, ev.value)

    # pageHandler(): touch on a page button -> switch page, then clear the button.
    def page_handler(self, addr, value):
        if value == TOUCH_ON and addr in PAGE_BUTTONS:
            page = PAGE_BUTTONS[addr]
            # ack=False: don't read the write ACK, so an incoming touch frame is
            # never consumed as the ACK while this loop owns the serial stream.
            self.dwin.goto_page(page, ack=False)
            self.dwin.write_single_reg(addr, 0x0000, ack=False)   # consume press
            self.dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
            self.page_index = page
            print(f"page button 0x{addr:04X} -> page {page}")

    # outputHandler(): touch on an output button -> toggle that output, then clear.
    def output_handler(self, addr, value):
        if value == TOUCH_ON:
            idx = addr - 0x0010                       # output index 0..15
            # state = not self.outputs.get(addr, False)
            state = self.dwin.read_single_reg(idx)
            self.outputs[addr] = ~state
            # TODO: drive the real output here -- GPIO pin, Modbus coil, etc.
            self.dwin.write_single_reg(addr, 0x0000, ack=False)   # consume press
            self.dwin.write_single_reg(idx, ~state)
            print(f"output {idx} (0x{addr:04X}) -> {'ON' if state else 'OFF'}")


def main():
    ap = argparse.ArgumentParser(
        description="DWIN touch events -> page switching / output toggling")
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--debug", action="store_true",
                    help="print every TX/RX frame as hex (diagnose comms)")
    args = ap.parse_args()

    dwin = DwinLCD(args.port, args.baud, debug=args.debug)
    if not dwin.is_connected():
        print("[warn] DWIN not responding -- check wiring / baud / UART setup")

    ui = DwinUI(dwin)
    print("Waiting for touch events (Ctrl-C to stop)...")
    last_test = 0.0
    try:
        while True:
            # blocks up to 0.2 s waiting for a frame, then returns None.
            ev = dwin.read_event(timeout=0.2)
            if ev is not None:
                ui.handle_frame(ev)
            else:
            # No touch this cycle. Every ~3 s prove the Pi -> HMI direction.
            # IMPORTANT: a touch upload (HMI -> Pi) needs only the RX wire, so
            # it can keep working even when the Pi TX line is dead. A page
            # switch / write needs the TX line, so test that separately:
            #   * buzzer beep  -> audible, needs NO display/VP project config
            #   * read 0x0014  -> round trip (Pi TX -> HMI -> Pi RX); getting
            #                     a reply proves writes also reach the panel.
                now = time.monotonic()
                if now - last_test >= 3.0:
                    last_test = now
                    dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
                    try:
                        # NB: read flushes pending RX, so run it while idle.
                        page = dwin.read_single_reg(0x0014)  # 0x0014 = cur page
                        print(f"[TX test] OK - HMI reachable, current page={page}")
                    except DwinTimeout:
                        print("[TX test] FAIL - no reply. Pi TX -> DWIN RX is not "
                                "getting through (wire / level-shifter / baud).")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        dwin.close()


if __name__ == "__main__":
    main()
