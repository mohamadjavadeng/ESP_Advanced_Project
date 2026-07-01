#!/usr/bin/env python3
"""
dwin_hmi_app.py -- two-page DWIN HMI controller for the excavation monitor.

What it does (exactly the flow asked for):

  PAGE 1 (main) -- the Pi WRITES to the panel:
      * current depth   -> VP 0x0001  as TEXT
      * driver name     -> VP 0x0200  as TEXT (max 20 chars)
      * target depth    -> VP 0x0300  -- every 20 s the program asks you on the
                           console for a target depth and writes it here.

  PAGE 2 (settings) -- the panel AUTO-UPLOADS to the Pi ("data auto-upload"
  mode), the Pi just caches whatever the panel pushes:
      * beam length     <- VP 0x0012  (4-char TEXT)
      * stick length    <- VP 0x0016  (TEXT)
      * Wi-Fi SSID      <- VP 0x0330  (TEXT)
      * Wi-Fi password  <- VP 0x0350  (TEXT)

  Control frames the panel pushes (DGUS "touch -> variable auto-upload"):
      * 0x0030 == 0x0022  -> panel is now on the SETTINGS page (page 2)
      * 0x0011 == 0x0022  -> SAVE  the cached settings
      * 0x0010 == 0x0022  -> CANCEL, go back to the main page

The current depth (and, if present, the driver name) is read from the
sensor_receiver.py service over HTTP (GET /status). If that service is not
reachable the depth is shown as "--.--" and the --driver-name argument is used.

Three things run at once, so a blocking console prompt never stops the panel
being serviced:
  * main thread  -- the event loop: reads auto-upload frames, dispatches them.
  * push thread  -- every --depth-interval s writes depth + driver name.
  * input thread -- every --target-interval s (20 s) asks you for target depth.
The DwinLCD driver is internally locked, so the three share one serial port
safely. Every write here uses ack=False so a write's ACK is never mistaken for
(and never eats) an incoming auto-upload frame -- see dwin_page_example.py.

!! ADDRESS WARNING !!
  VPs 0x0010-0x0017 and 0x0014 are the DGUS *system* RTC / current-page region.
  The addresses above (0x0010, 0x0012, 0x0014-ish, 0x0016) overlap it. This is
  fine ONLY if your DGUS project genuinely maps these user variables there; the
  usual safe range for user data is >= 0x1000. If beam/stick/save/cancel misbehave
  this overlap is the first thing to check.

Run from the raspberry_pi/ folder (so `import dwin_lcd` resolves):
    python3 dwin_hmi_app.py --port /dev/serial0 --baud 115200 \
                            --driver-name "A. OPERATOR" \
                            --status-url http://127.0.0.1:8080/status
"""

import argparse
import json
import os
import queue
import threading
import time
import urllib.request

from dwin_lcd import DwinLCD, BuzzerDuration

# --------------------------------------------------------------- VP address map
# Page 1 (Pi -> panel)
VP_DEPTH_TEXT   = 0x0001   # current depth, written as TEXT
VP_DRIVER_NAME  = 0x0200   # driver name, TEXT, max 20 chars
VP_TARGET_DEPTH = 0x0300   # target-depth field, Pi writes every 20 s

# Page 2 (panel -> Pi, auto-upload)
VP_BEAM_LEN  = 0x0012      # beam length, 4-char TEXT
VP_STICK_LEN = 0x0016      # stick length, TEXT
VP_SSID      = 0x0330      # Wi-Fi SSID, TEXT
VP_PASSWORD  = 0x0350      # Wi-Fi password, TEXT
SETTINGS_VPS = (VP_BEAM_LEN, VP_STICK_LEN, VP_SSID, VP_PASSWORD)

# Control / status frames (panel -> Pi)
VP_PAGE_FLAG  = 0x0030     # == TRIGGER -> panel is on the settings page
VP_SAVE_BTN   = 0x0011     # == TRIGGER -> save settings
VP_CANCEL_BTN = 0x0010     # == TRIGGER -> cancel, back to main

TRIGGER  = 0x0022          # the value each control VP carries when active
CMD_READ = 0x83            # auto-upload frames look like a 0x83 read response

# TEXT field widths (bytes) the Pi writes; pad so old longer text is cleared.
DEPTH_LEN  = 8
NAME_LEN   = 20
TARGET_LEN = 8


# --------------------------------------------------------------------- helpers
def put_text(dwin, addr, text, pad, ack=False):
    """Write an ASCII string to a TEXT VP, padded/truncated to `pad` bytes.

    ack=False (the default here) skips reading the write ACK, so this never
    consumes an incoming auto-upload frame. Padding with spaces overwrites any
    leftover characters from a previously longer string.
    """
    buf = text.encode("ascii", "replace")[:pad].ljust(pad, b"\x20")
    dwin.write_data(addr, buf, ack=ack)


def event_bytes(ev):
    """Raw data bytes carried by an auto-upload frame (handles N-word payloads).

    Frame: 5A A5 <len> 83 <addrHi> <addrLo> <nWords> <data...>, so the data is
    nWords*2 bytes starting at offset 7.
    """
    f = ev.raw
    if len(f) < 8:
        return b""
    nwords = f[6]
    return bytes(f[7:7 + 2 * nwords])


def decode_text(data):
    """Decode a DGUS text payload: ASCII up to the 0x0000 / 0xFFFF terminator."""
    out = []
    for b in data:
        if b in (0x00, 0xFF):
            break
        out.append(b)
    return bytes(out).decode("ascii", "replace").strip()


# ------------------------------------------------------------------- the app
class HmiApp:
    def __init__(self, dwin, args):
        self.dwin = dwin
        self.args = args
        self.stop = threading.Event()
        self.in_settings = False
        # Latest value pushed by the panel for each settings VP. Seeded from the
        # saved file so a SAVE with no edits keeps the previous values.
        self.cache = {vp: "" for vp in SETTINGS_VPS}
        self.cache_lock = threading.Lock()
        self.settings_path = args.settings_file
        # Page-1 values to write, refreshed by the status thread (which never
        # touches the serial port -- only the main loop writes to the panel).
        self.depth_txt = "--.--"
        self.driver_name = args.driver_name
        # Lines typed on the console, fed by the stdin thread, drained by main.
        self.input_q = queue.Queue()
        self._load_settings()

    # ----- persistence ------------------------------------------------------
    def _load_settings(self):
        try:
            with open(self.settings_path) as f:
                s = json.load(f)
        except (OSError, ValueError):
            return
        self.cache[VP_BEAM_LEN]  = str(s.get("beam_len_mm", ""))
        self.cache[VP_STICK_LEN] = str(s.get("stick_len_mm", ""))
        self.cache[VP_SSID]      = str(s.get("wifi_ssid", ""))
        self.cache[VP_PASSWORD]  = str(s.get("wifi_password", ""))
        print(f"[init] loaded saved settings from {self.settings_path}")

    # ----- HTTP to sensor_receiver.py --------------------------------------
    def _fetch_status(self):
        if not self.args.status_url:
            return None
        try:
            with urllib.request.urlopen(self.args.status_url, timeout=0.5) as r:
                return json.loads(r.read().decode())
        except Exception:
            return None

    def _depth_and_name(self):
        st = self._fetch_status()
        if st is None:
            return "--.--", self.args.driver_name
        depth = st.get("depth_m")
        depth_txt = f"{depth:.2f}" if isinstance(depth, (int, float)) else "--.--"
        name = st.get("driver_name") or self.args.driver_name
        return depth_txt, name

    def _post_config(self, beam_mm, stick_mm):
        """Best-effort: push beam/stick (mm -> m) to sensor_receiver /config."""
        if not self.args.status_url:
            return
        url = self.args.status_url.rsplit("/", 1)[0] + "/config"
        try:
            body = json.dumps({"L1": float(beam_mm) / 1000.0,
                               "L2": float(stick_mm) / 1000.0}).encode()
        except ValueError:
            print("[save] beam/stick not numeric, skipped /config push")
            return
        try:
            req = urllib.request.Request(
                url, data=body, headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=0.5).read()
            print(f"[save] geometry pushed to {url}")
        except Exception as e:
            print(f"[save] could not push geometry: {e}")

    # ----- event handling (main thread) ------------------------------------
    def handle_event(self, ev):
        # Only act on auto-upload / read frames; ignore write ACKs (0x82).
        if ev.cmd != CMD_READ:
            return
        addr, val = ev.addr, ev.value

        # page indicator (status, not a button -> don't write back, edge-detect)
        if addr == VP_PAGE_FLAG:
            if val == TRIGGER and not self.in_settings:
                self.in_settings = True
                print("[page] panel entered SETTINGS (page 2)")
            elif val != TRIGGER and self.in_settings:
                self.in_settings = False
            return

        # SAVE / CANCEL buttons: clear the press so it does not retrigger.
        if addr == VP_SAVE_BTN and val == TRIGGER:
            self.dwin.write_single_reg(addr, 0x0000, ack=False)
            self._do_save()
            return
        if addr == VP_CANCEL_BTN and val == TRIGGER:
            self.dwin.write_single_reg(addr, 0x0000, ack=False)
            self._do_cancel()
            return

        # settings text fields, pushed by the panel in auto-upload mode
        if addr in SETTINGS_VPS:
            text = decode_text(event_bytes(ev))
            with self.cache_lock:
                self.cache[addr] = text
            shown = text if addr != VP_PASSWORD else "*" * len(text)
            print(f"[recv] 0x{addr:04X} = '{shown}'")

    def _do_save(self):
        with self.cache_lock:
            beam  = self.cache.get(VP_BEAM_LEN, "")
            stick = self.cache.get(VP_STICK_LEN, "")
            ssid  = self.cache.get(VP_SSID, "")
            pw    = self.cache.get(VP_PASSWORD, "")
        print(f"[save] beam={beam!r}mm stick={stick!r}mm "
              f"ssid={ssid!r} pw={'*' * len(pw)}")
        settings = {"beam_len_mm": beam, "stick_len_mm": stick,
                    "wifi_ssid": ssid, "wifi_password": pw}
        try:
            with open(self.settings_path, "w") as f:
                json.dump(settings, f, indent=2)
            print(f"[save] written -> {self.settings_path}")
        except OSError as e:
            print(f"[save] FAILED to write {self.settings_path}: {e}")
        self._post_config(beam, stick)
        # NOTE: actually applying the Wi-Fi SSID/password to the OS (wpa_supplicant
        # / NetworkManager) is system-specific -- do it here if you need it.
        self.dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
        self.dwin.goto_page(self.args.main_page, ack=False)
        self.in_settings = False
        print("[save] done -> back to main page")

    def _do_cancel(self):
        print("[cancel] discarding edits -> back to main page")
        # drop any uncommitted edits: reload the last saved values into the cache
        with self.cache_lock:
            self._load_settings()
        self.dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
        self.dwin.goto_page(self.args.main_page, ack=False)
        self.in_settings = False

    # ----- helper threads (NEITHER of these touches the serial port) -------
    def _status_poll_loop(self):
        """Refresh the cached depth + driver name from /status (HTTP only)."""
        while not self.stop.is_set():
            self.depth_txt, self.driver_name = self._depth_and_name()
            if self.stop.wait(self.args.depth_interval):
                break

    def _stdin_loop(self):
        """Read console lines into a queue so input() never blocks the panel."""
        while not self.stop.is_set():
            try:
                line = input()
            except EOFError:
                return
            self.input_q.put(line)

    # ----- work done by the main (serial-owning) thread --------------------
    def _push_depth_name(self):
        try:
            put_text(self.dwin, VP_DEPTH_TEXT, self.depth_txt, DEPTH_LEN)
            put_text(self.dwin, VP_DRIVER_NAME, self.driver_name, NAME_LEN)
        except Exception as e:
            print(f"[push] write failed: {e}")

    def _apply_target(self, raw):
        raw = raw.strip()
        if not raw:
            return
        try:
            val = float(raw)
        except ValueError:
            print(f"[target] '{raw}' is not a number -- ignored")
            return
        txt = f"{val:.2f}"
        # Written as TEXT to the field VP. If 0x0300 is a NUMERIC variable
        # instead, use: self.dwin.write_single_reg(VP_TARGET_DEPTH,
        #                                           int(round(val * 100)), ack=False)
        try:
            put_text(self.dwin, VP_TARGET_DEPTH, txt, TARGET_LEN)
            print(f"[target] wrote {txt} m -> 0x{VP_TARGET_DEPTH:04X}")
        except Exception as e:
            print(f"[target] write failed: {e}")

    def _drain_input(self):
        while True:
            try:
                line = self.input_q.get_nowait()
            except queue.Empty:
                return
            self._apply_target(line)

    # ----- main loop: the ONLY thread that uses the serial port ------------
    def run(self):
        # Helper threads do HTTP and console I/O; the main loop alone reads
        # frames AND writes to the panel -- so nothing ever fights for the
        # serial lock and the depth keeps refreshing while you type a target.
        threading.Thread(target=self._status_poll_loop, daemon=True).start()
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        print(f"Servicing HMI (Ctrl-C to stop). Target-depth prompt every "
              f"{self.args.target_interval:.0f}s -- just type a number + Enter.")

        last_push = 0.0
        last_prompt = time.monotonic()   # first prompt one interval from now
        try:
            while True:
                # short read so the loop also services pushes/prompts promptly
                ev = self.dwin.read_event(timeout=0.05)
                if ev is not None:
                    self.handle_event(ev)

                now = time.monotonic()
                if now - last_push >= self.args.depth_interval:
                    last_push = now
                    self._push_depth_name()
                if now - last_prompt >= self.args.target_interval:
                    last_prompt = now
                    print("[target] type TARGET DEPTH in metres + Enter: ",
                          flush=True)
                self._drain_input()
        except KeyboardInterrupt:
            print("\nstopping...")
        finally:
            self.stop.set()


def main():
    ap = argparse.ArgumentParser(
        description="Two-page DWIN HMI controller (depth/name/target + settings)")
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--debug", action="store_true",
                    help="print every TX/RX frame as hex")
    ap.add_argument("--status-url", default="http://127.0.0.1:8080/status",
                    help="sensor_receiver /status URL (empty to disable HTTP)")
    ap.add_argument("--driver-name", default="DRIVER",
                    help="fallback driver name when /status has none")
    ap.add_argument("--main-page", type=int, default=0,
                    help="page index of the main screen (for SAVE/CANCEL return)")
    ap.add_argument("--depth-interval", type=float, default=1.0,
                    help="seconds between depth/name writes")
    ap.add_argument("--target-interval", type=float, default=20.0,
                    help="seconds between target-depth prompts")
    ap.add_argument("--settings-file",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "hmi_settings.json"))
    args = ap.parse_args()

    dwin = DwinLCD(args.port, args.baud, debug=args.debug)
    if not dwin.is_connected():
        print("[warn] DWIN not responding -- check wiring / baud / UART setup")

    app = HmiApp(dwin, args)
    try:
        app.run()
    finally:
        dwin.close()


if __name__ == "__main__":
    main()
