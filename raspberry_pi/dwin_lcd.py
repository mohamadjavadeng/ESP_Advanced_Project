#!/usr/bin/env python3
"""
dwin_lcd.py -- Raspberry Pi (pyserial) driver for DWIN T5L DGUS II LCDs.

A faithful Python port of the Arduino library in
`documents for learning/DWINLCD` (DWIN_LCD.h / DWIN_LCD.cpp), so every method
maps 1:1 (camelCase -> snake_case):

    Arduino                     Python
    -----------------------     ---------------------------
    begin(baud)                 DwinLCD(port, baud)  (opens on construct)
    isConnected()               is_connected()
    nextPage()/previousPage()   next_page() / previous_page()
    gotoPage(p)                 goto_page(p)
    writeSingleReg(a,v)         write_single_reg(a, v)
    writeData(a,bytes,len)      write_data(a, data)   /  write_text(a, str)
    setSingleBit/resetSingleBit set_single_bit / reset_single_bit
    readSingleReg(a)            read_single_reg(a)
    readSingleBit(a,b)          read_single_bit(a, b)
    readReg(a,n,buf)            read_reg(a, n) -> bytes
    readRTC()/writeRTC(...)     read_rtc() / write_rtc(...)
    backlight()                 backlight() -> (value, current)
    buzzer(dur)                 buzzer(dur)

Protocol (DGUS II):
    frame = 5A A5 <len> <cmd> <addrHi> <addrLo> <data...>
    len   = number of bytes AFTER the len byte (cmd + addr2 + data)
    cmd   = 0x82 write, 0x83 read

Differences from the Arduino version (improvements, not behaviour changes):
  * Replies are read as whole frames (sync on 5A A5, then read `len` bytes) and
    the read reply's address is verified -- so an asynchronous touch-press frame
    from the panel never gets mis-parsed as a register read result.
  * Every transaction is guarded by a lock, so a status-polling thread and a
    touch-handling thread can share one DwinLCD safely.
  * Read timeouts raise DwinTimeout instead of returning garbage.

Raspberry Pi wiring / setup:
  * DWIN UART2 is 8N1. Pi <-> DWIN: Pi TX(GPIO14) -> DWIN RX, Pi RX(GPIO15) <-
    DWIN TX, common GND. Most DWIN panels are 5V TTL -> use a level shifter (at
    least on the Pi RX line); the Pi GPIO is NOT 5V tolerant.
  * Enable the hardware UART and free it from the login console:
        sudo raspi-config  ->  Interface Options -> Serial Port
            "login shell over serial?"  NO
            "serial port hardware enabled?"  YES
    then reboot. The port is /dev/serial0.
  * pip install pyserial

Quick test on the Pi:
    python3 dwin_lcd.py --port /dev/serial0 --baud 115200
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import serial   # pip install pyserial


# --------------------------------------------------------------------------- #
class BuzzerDuration(IntEnum):
    """Beep length -> value written to the buzzer register 0x00A0."""
    BUZZ_1SEC = 0x7D
    BUZZ_500MSEC = 0x3E
    BUZZ_250MSEC = 0x20


class Weekday(IntEnum):
    SUNDAY = 0
    MONDAY = 1
    TUESDAY = 2
    WEDNESDAY = 3
    THURSDAY = 4
    FRIDAY = 5
    SATURDAY = 6


class DwinError(Exception):
    pass


class DwinTimeout(DwinError):
    """No (matching) reply arrived from the panel in time."""


@dataclass
class RtcTime:
    year: int
    month: int
    day: int
    weekday: int
    hour: int
    minute: int
    second: int


@dataclass
class DwinEvent:
    """One frame received from the panel (touch auto-upload, or a write ACK).

    For a touch auto-upload / read frame (cmd 0x83), `addr` and `value` are the
    VP and its 16-bit value. For other frames (e.g. the 0x82 write ACK) they are
    0 -- callers usually act only when `cmd == 0x83`.
    """
    cmd: int
    addr: int
    value: int
    raw: bytes


# DGUS system variable-pointer (VP) addresses used by the library.
_VP_PAGE_NOW = 0x0014     # current page (read)
_VP_PAGE_SET = 0x0084     # page switch: write 0x5A01 then page (2 bytes)
_VP_RTC = 0x0010          # RTC data
_VP_BACKLIGHT = 0x0031    # backlight (read: value, current)
_VP_BUZZER = 0x00A0       # buzzer

_HDR0, _HDR1 = 0x5A, 0xA5
_CMD_WRITE, _CMD_READ = 0x82, 0x83


class DwinLCD:
    """Driver for one DWIN T5L DGUS II panel on a serial port."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 timeout: float = 0.4, open_now: bool = True,
                 debug: bool = False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.debug = debug   # True -> dump every frame sent/received
        self._lock = threading.RLock()
        self._ser: Optional[serial.Serial] = None
        # RTC fields are populated by read_rtc() (mirrors the Arduino members).
        self.year = self.month = self.day = self.weekday = 0
        self.hour = self.minute = self.second = 0
        self.backlight_value = 0
        self.backlight_current = 0
        if open_now:
            self.begin()

    # ----- lifecycle -------------------------------------------------------- #
    def begin(self, baud: Optional[int] = None) -> None:
        """Open (or reopen) the serial port. Mirrors Arduino begin(baud)."""
        if baud is not None:
            self.baud = baud
        if self._ser is not None:
            self._ser.close()
        # read timeout is set per-call in _read_frame; keep a small base here.
        self._ser = serial.Serial(self.port, self.baud, timeout=0.05,
                                  bytesize=8, parity="N", stopbits=1)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "DwinLCD":
        if self._ser is None:
            self.begin()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ----- low-level framing ----------------------------------------------- #
    @staticmethod
    def _build(cmd: int, addr: int, data: bytes = b"") -> bytes:
        body = bytes([cmd, (addr >> 8) & 0xFF, addr & 0xFF]) + bytes(data)
        temp_list = [_HDR0, _HDR1, len(body)]
        for item in body:
            temp_list.append(item)
        return temp_list

    def _dump(self, tag: str, raw) -> None:
        """Print one frame as hex when debug is on (the --debug frame trace).

        This is the tool for "the panel button does nothing": if a press
        produces no RX line here, the panel is not auto-uploading at all.
        """
        if self.debug:
            print(f"[dwin] {tag} {bytes(raw).hex(' ').upper()}", flush=True)

    def _read_frame(self, timeout: float) -> Optional[bytes]:
        """Read one full DGUS frame (sync on 5A A5, then `len` bytes)."""
        ser = self._ser
        end = time.monotonic() + timeout
        prev = -1
        # 1) sync to the 5A A5 header
        while time.monotonic() < end:
            b = ser.read(1)
            if not b:
                continue
            if prev == _HDR0 and b[0] == _HDR1:
                break
            prev = b[0]
        else:
            return None
        # 2) length byte
        lb = ser.read(1)
        if len(lb) != 1:
            return None
        length = lb[0]
        # 3) the rest of the frame
        body = ser.read(length)
        if len(body) != length:
            return None
        out = bytes([_HDR0, _HDR1, length]) + body
        self._dump("RX", out)
        return out

    def _read_reply(self, addr: int, timeout: float, sink=None) -> bytes:
        """Return the next 0x83 read-reply whose address == addr.

        Skips unrelated frames (e.g. async touch events). Raises on timeout.
        `sink`, if given, is called with each skipped frame as a DwinEvent so an
        event loop that polls a VP does not silently lose a button press that
        arrived mid-poll.
        """
        end = time.monotonic() + timeout
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                raise DwinTimeout(f"no reply for VP 0x{addr:04X}")
            f = self._read_frame(remaining)
            if not f:
                continue
            if len(f) >= 9 and f[3] == _CMD_READ \
                    and f[4] == ((addr >> 8) & 0xFF) and f[5] == (addr & 0xFF):
                return f
            if sink is not None:
                sink(self._to_event(f))

    def _drain_ack(self) -> None:
        """Best-effort read of the write ACK (5A A5 03 82 4F 4B); ignore it."""
        self._read_frame(0.1)

    def _send(self, frame: bytes) -> None:
        """Write a frame (used for 0x82 writes; reads go through _query)."""
        # _build() returns a list of ints; normalise to bytes for the wire.
        raw = bytes(frame)
        self._dump("TX", raw)
        self._ser.write(raw)
        self._ser.flush()   # block until the bytes are actually clocked out

    def _query(self, addr: int, n_words: int, timeout: Optional[float] = None,
               flush: bool = True, sink=None) -> bytes:
        """Send a 0x83 read and return the matching reply frame.

        flush=True (default) drops whatever is already buffered first, which is
        right for a standalone read. Inside an event loop pass flush=False plus a
        `sink` so queued/interleaved touch frames are handed back instead of
        discarded.
        """
        to = self.timeout if timeout is None else timeout
        with self._lock:
            if flush:
                self._ser.reset_input_buffer()
            self._send(self._build(_CMD_READ, addr, bytes([n_words])))
            return self._read_reply(addr, to, sink=sink)

    # ----- connection ------------------------------------------------------- #
    def is_connected(self) -> bool:
        """True if the panel answers a read of the backlight register."""
        try:
            self._query(_VP_BACKLIGHT, 1, min(self.timeout, 0.3))
            return True
        except (DwinTimeout, serial.SerialException):
            return False

    # ----- pages ------------------------------------------------------------ #
    def _switch_page(self, page: int, ack: bool = True) -> None:
        # write 0x5A 0x01 <pageHi> <pageLo> to 0x0084
        data = bytes([0x5A, 0x01, (page >> 8) & 0xFF, page & 0xFF])
        with self._lock:
            self._send(self._build(_CMD_WRITE, _VP_PAGE_SET, data))
            if ack:
                self._drain_ack()

    def goto_page(self, page: int, ack: bool = True) -> None:
        """Jump to a specific page number.

        ack=False skips reading the write ACK -- use it inside an event loop
        (read_event) so an incoming touch frame is not consumed as the ACK.
        """
        self._switch_page(page, ack=ack)

    def next_page(self) -> None:
        """Show current page + 1 (reads 0x0014 to learn the current page)."""
        self._switch_page((self.read_single_reg(_VP_PAGE_NOW) + 1) & 0xFFFF)

    def previous_page(self) -> None:
        self._switch_page((self.read_single_reg(_VP_PAGE_NOW) - 1) & 0xFFFF)

    # ----- register writes -------------------------------------------------- #
    def write_single_reg(self, addr: int, value: int, ack: bool = True) -> None:
        """Write one 16-bit word to a VP.

        ack=False skips reading the write ACK (use inside an event loop).
        """
        data = bytes([(value >> 8) & 0xFF, value & 0xFF])
        with self._lock:
            self._send(self._build(_CMD_WRITE, addr, data))
            if ack:
                self._drain_ack()

    def write_data(self, addr: int, data, ack: bool = True) -> None:
        """Write raw bytes (e.g. a text string) starting at a VP."""
        if isinstance(data, str):
            data = data.encode("ascii", "replace")
        with self._lock:
            self._send(self._build(_CMD_WRITE, addr, bytes(data)))
            if ack:
                self._drain_ack()

    def write_text(self, addr: int, text: str, pad: Optional[int] = None,
                   fill: int = 0x20, encoding: str = "ascii") -> None:
        """Write a string to a text VP.

        pad: if set, pad/truncate to `pad` bytes with `fill` (default space) so
        leftover characters from a previous, longer string are overwritten.
        """
        buf = text.encode(encoding, "replace")
        if pad is not None:
            buf = buf[:pad].ljust(pad, bytes([fill]))
        self.write_data(addr, buf)

    # ----- bit ops (read-modify-write a word VP) ---------------------------- #
    def set_single_bit(self, addr: int, bit: int) -> None:
        with self._lock:
            self.write_single_reg(addr, self.read_single_reg(addr) | (1 << bit))

    def reset_single_bit(self, addr: int, bit: int) -> None:
        with self._lock:
            self.write_single_reg(addr,
                                  self.read_single_reg(addr) & ~(1 << bit))

    # ----- register reads --------------------------------------------------- #
    def read_single_reg(self, addr: int) -> int:
        """Read one 16-bit word from a VP."""
        f = self._query(addr, 1)
        return (f[7] << 8) | f[8]

    def read_single_bit(self, addr: int, bit: int) -> bool:
        return bool(self.read_single_reg(addr) & (1 << bit))

    def read_reg(self, addr: int, n_words: int, timeout: Optional[float] = None,
                 flush: bool = True, sink=None) -> bytes:
        """Read `n_words` 16-bit words; returns the raw 2*n_words data bytes.

        flush/sink are passed through to _query -- use flush=False + sink from an
        event loop so a touch frame arriving mid-read is not thrown away.
        """
        f = self._query(addr, n_words, timeout=timeout, flush=flush, sink=sink)
        return f[7:7 + 2 * n_words]

    # ----- touch events (unsolicited frames from the panel) ----------------- #
    @staticmethod
    def _to_event(f: bytes) -> DwinEvent:
        """Wrap a raw frame as a DwinEvent (addr/value filled for 0x83 only)."""
        cmd = f[3] if len(f) > 3 else 0
        addr = value = 0
        if cmd == _CMD_READ and len(f) >= 9:
            addr = (f[4] << 8) | f[5]
            value = (f[7] << 8) | f[8]
        return DwinEvent(cmd, addr, value, f)

    def read_event(self, timeout: float = 0.1) -> Optional[DwinEvent]:
        """Read ONE unsolicited frame the panel pushed (touch auto-upload).

        Unlike the read_* methods this does NOT flush input, so touch frames are
        never lost. Returns None on timeout. A DGUS "touch -> variable data
        auto-upload" button sends a 0x83 frame
            5A A5 06 83 <addrHi> <addrLo> 01 <valHi> <valLo>
        so check `ev.cmd == 0x83` and use `ev.addr` / `ev.value`. When you write
        back from the handler (e.g. to clear the button), pass ack=False so this
        loop keeps owning the incoming stream.
        """
        with self._lock:
            f = self._read_frame(timeout)
        if not f:
            return None
        return self._to_event(f)

    # ----- RTC -------------------------------------------------------------- #
    def read_rtc(self) -> RtcTime:
        """Read the panel RTC (VP 0x0010). Also sets self.year/month/... ."""
        d = self.read_reg(_VP_RTC, 4)   # 8 data bytes; last is unused
        self.year, self.month, self.day, self.weekday = d[0], d[1], d[2], d[3]
        self.hour, self.minute, self.second = d[4], d[5], d[6]
        return RtcTime(self.year, self.month, self.day, self.weekday,
                       self.hour, self.minute, self.second)

    def write_rtc(self, day: int, month: int, year: int, hour: int,
                  minute: int, second: int, weekday: int) -> None:
        """Set the panel RTC. Argument order matches the Arduino writeRTC().

        Frame writes year,month,day,weekday,hour,minute,second to 0x0010.
        (year is the 2-digit year, e.g. 24 for 2024.)
        """
        data = bytes([year, month, day, int(weekday), hour, minute, second, 0])
        with self._lock:
            self._send(self._build(_CMD_WRITE, _VP_RTC, data))
            self._drain_ack()

    # ----- misc ------------------------------------------------------------- #
    def backlight(self) -> Tuple[int, int]:
        """Read the backlight register: returns (value, current)."""
        d = self.read_reg(_VP_BACKLIGHT, 1)
        self.backlight_value, self.backlight_current = d[0], d[1]
        return self.backlight_value, self.backlight_current

    def buzzer(self, duration: BuzzerDuration = BuzzerDuration.BUZZ_1SEC,
               ack: bool = True) -> None:
        """Beep the panel buzzer for the given duration."""
        with self._lock:
            self._send(self._build(_CMD_WRITE, _VP_BUZZER,
                                   bytes([0x00, int(duration)])))
            if ack:
                self._drain_ack()


# --------------------------------------------------------------------------- #
def _demo():
    """Bench self-test, mirroring the Arduino RegisterWrite example.

    Project VP map (see DEPLOYMENT_GUIDE.md): 0x1000 depth, 0x1002 target,
    0x1010 alarm flag, 0x2000 text, 0x5000 operator offset (DWIN -> Pi).
    """
    import argparse
    ap = argparse.ArgumentParser(description="DWIN LCD Python driver self-test")
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    with DwinLCD(args.port, args.baud) as dwin:
        print("connected:", dwin.is_connected())
        dwin.write_text(0x2000, "Hello", pad=16)        # text VP
        dwin.write_single_reg(0x1000, 1420)             # e.g. depth = 1.420 m
        dwin.set_single_bit(0x1010, 0)                  # raise an alarm icon bit
        dwin.buzzer(BuzzerDuration.BUZZ_250MSEC)
        try:
            print("page now:", dwin.read_single_reg(_VP_PAGE_NOW))
        except DwinTimeout as e:
            print("read failed:", e)


if __name__ == "__main__":
    _demo()
