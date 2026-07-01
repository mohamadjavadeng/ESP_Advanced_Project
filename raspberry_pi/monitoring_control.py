#!/usr/bin/env python3
"""
monitoring_control.py -- FINAL integrated Raspberry Pi controller for the
excavation depth-monitoring product.

ONE process ties together what used to be separate scripts:

  1. SENSOR INGEST (was sensor_receiver.py) -- a Flask server the two ESP32
     units POST to:
        POST /ingest    {"id":"stick"|"beam","angle_deg":..,"mpu_ok":..,"seq":..}
        POST /location  {"id":"stick","gps_ok":..,"lat":..,"lon":..,...}
     It computes  depth = L1*sin(boom) + L2*sin(stick)  and serves GET /status.

  2. DWIN HMI (was dwin_hmi_app.py) -- the ONLY thread that owns the serial
     port. It WRITES current depth (0x0001), driver name (0x0200) and target
     depth (0x0300) to the panel, reads the settings page (beam/stick/wifi via
     auto-upload), and drives the ALARMS: OVER-DIG (VP 0x0401) when depth >=
     target, HSE/person (VP 0x0402) from the camera thread, and beeps the buzzer
     whenever EITHER is active (silent only when both are clear). The over-dig
     alarm auto-clears once the operator lifts the bucket (depth < target - hyst).

  3. CLOUD SYNC (GEOMind / geobox SDK) -- a background thread that:
        * first run on a device: creates a VECTOR layer (+ one FEATURE) and a
          TABLE, then stores their ids in the state file; later runs reuse them
          (checks the state file, then get_*_by_name, then creates).
        * reads TARGET DEPTH from the feature (operator sets it in the GEOMind
          web UI) and feeds it to the alarm logic + the panel.
        * pushes current depth + all alarms + GPS into the feature fields.
        * appends a row to the table (heartbeat every --row-interval s, plus an
          extra row on every alarm edge / driver change).

  4. RFID -> DRIVER NAME -- an MFRC522 reader thread: on a card scan it looks the
     tag up in a CSV stored on the cloud (--driver-csv) and sets the active
     driver name (shown on the HMI, written to the cloud).

  5. CAMERA HSE ALARM (person_detector.py) -- a YOLOv4-tiny person-in-zone
     detector thread. When a person dwells in the watch-zone it sets
     runtime["hse_alarm"]; the HMI mirrors it to the panel + buzzer and the cloud
     pushes it. It clears automatically when the zone is empty.

  6. DRIVER SESSIONS -- the cloud thread opens a row in a <device>_sessions table
     on each driver change (start time + driver + UID), counts HSE / over-dig
     alarm rising edges during the session, and stamps the end time when the next
     driver signs in (or on shutdown).

The cloud and RFID threads NEVER touch the serial port -- they only update
shared state; the HMI thread reads that state and writes the panel, exactly like
dwin_hmi_app.py, so nothing fights for the serial lock. The excavator keeps
working (depth, alarm, buzzer) even with NO network -- cloud/RFID failures are
caught and retried, and the last target depth is persisted to the state file so
the alarm still has a threshold after an offline restart.

Run from the raspberry_pi/ folder (so `import dwin_lcd` resolves):
    pip install flask pyserial geobox tqdm mfrc522 opencv-python-headless numpy
    python3 monitoring_control.py \
        --http-port 8080 --dwin-port /dev/serial0 --dwin-baud 115200 \
        --geomind-host https://app.geo-mind.ai \
        --geomind-user PDO@excavator1 --geomind-pass PDO@excavator1 \
        --device-id excavator1 --driver-csv drivers.csv

The geobox SDK defaults its host to https://api.geobox.ir; we point it at
https://app.geo-mind.ai purely via the GeoboxClient(host=...) argument -- no SDK
source edit is needed.
"""

import argparse
import csv
import io
import json
import logging
import math
import os
import threading
import time
from datetime import datetime, timezone

from flask import Flask, jsonify, request

from dwin_lcd import DwinLCD, BuzzerDuration

# --- optional deps: the program runs without them; the matching subsystem just
#     degrades gracefully (so this file is also testable on a dev PC). ---------
try:
    from geobox import GeoboxClient
    from geobox.enums import LayerType, FieldType
    _GEOBOX_OK, _GEOBOX_ERR = True, None
except Exception as _e:                      # ImportError or any load error
    GeoboxClient = LayerType = FieldType = None
    _GEOBOX_OK, _GEOBOX_ERR = False, _e

try:
    from mfrc522 import SimpleMFRC522         # pulls in RPi.GPIO + spidev (Pi only)
    _MFRC522_OK, _MFRC522_ERR = True, None
except Exception as _e:
    SimpleMFRC522 = None
    _MFRC522_OK, _MFRC522_ERR = False, _e

try:
    import cv2                                # OpenCV, for the camera person-detector
    _CV2_OK, _CV2_ERR = True, None
except Exception as _e:                       # not installed / no display libs
    cv2 = None
    _CV2_OK, _CV2_ERR = False, _e


# --------------------------------------------------------------- VP address map
# Page 1 (Pi -> panel)
VP_DEPTH_TEXT    = 0x0001   # current depth, written as TEXT
VP_DRIVER_NAME   = 0x0200   # driver name, TEXT, max 20 chars
VP_TARGET_DEPTH  = 0x0300   # target-depth field (from the cloud)
VP_OVERDIG_ALARM = 0x0401   # over-dig alarm flag: 1 = alarm, 0 = clear  (NEW)
VP_HSE_ALARM     = 0x0402   # HSE/person alarm flag: 1 = alarm, 0 = clear (NEW; --hse-vp)

# Page 2 (panel -> Pi, auto-upload)
VP_BEAM_LEN  = 0x0012      # beam length, TEXT (mm)
VP_STICK_LEN = 0x2000      # stick length, TEXT (mm)
VP_SSID      = 0x0330      # Wi-Fi SSID, TEXT
VP_PASSWORD  = 0x0350      # Wi-Fi password, TEXT
SETTINGS_VPS = (VP_BEAM_LEN, VP_STICK_LEN, VP_SSID, VP_PASSWORD)

# Control / status frames (panel -> Pi)
VP_PAGE_FLAG  = 0x0030     # == TRIGGER -> panel is on the settings page
VP_SAVE_BTN   = 0x0011     # == TRIGGER -> save settings
VP_CANCEL_BTN = 0x0010     # == TRIGGER -> cancel, back to main

TRIGGER  = 0x0022          # value each control VP carries when active
CMD_READ = 0x83            # auto-upload frames look like a 0x83 read response

# TEXT field widths (bytes) the Pi writes; pad so old longer text is cleared.
DEPTH_LEN, NAME_LEN, TARGET_LEN = 8, 20, 8


# --------------------------------------------------------------------------- #
# All shared state lives here, guarded by a lock (Flask is multi-threaded and
# the HMI / cloud / RFID threads all read or write it).
_lock = threading.Lock()
_stop = threading.Event()
# Set by the RFID thread when it scans a tag missing from the cached directory;
# the cloud thread services it by re-downloading the driver CSV immediately.
_rfid_refresh = threading.Event()

state = {
    "stick": {"angle_deg": 0.0, "mpu_ok": False, "seq": -1, "ts": 0.0},
    "beam":  {"angle_deg": 0.0, "mpu_ok": False, "seq": -1, "ts": 0.0},
}
# Latest GNSS fix per unit (only units with a SIM7000G populate this -- the
# stick does by default; the beam stays all-zero unless it gets a modem too).
location = {
    "stick": {"gps_ok": False, "lat": 0.0, "lon": 0.0, "alt_m": 0.0,
              "speed_kph": 0.0, "sats": 0, "hacc_m": 0.0, "seq": -1, "ts": 0.0},
    "beam":  {"gps_ok": False, "lat": 0.0, "lon": 0.0, "alt_m": 0.0,
              "speed_kph": 0.0, "sats": 0, "hacc_m": 0.0, "seq": -1, "ts": 0.0},
}
cfg = {
    "L1": 1.1, "L2": 1.0,            # boom / stick lengths (m)
    "boom_sign": 1.0, "stick_sign": 1.0,
    "boom_offset_deg": 0.0, "stick_offset_deg": 0.0,
    "target_depth": 1.5,
    "stale_ms": 1500,                # an angle unit older than this is "stale"
    "gps_stale_ms": 11 * 60 * 1000,  # a GNSS fix older than this is gps_ok=false
}
# Live, non-sensor state shared with the cloud + HMI threads.
runtime = {
    "driver_name": "DRIVER", "driver_tag": "",
    "rfid_uid": "",                  # raw UID string of the last scanned tag
    "driver_present": False,         # True once a KNOWN driver is identified
    "over_dig_alarm": False,         # latched (with hysteresis) by the HMI thread
    "sensor_alarm": False,           # a depth sensor is dead/stale
    "hse_alarm": False,              # set by the camera person-detector thread
    "cloud_ok": False, "hmi_ok": False,
}

# Driver directory (RFID tag -> name); set in main(), used by /rfid + RFID thread.
DIRECTORY = None


# --------------------------------------------------------------- persistence --
# A single JSON file remembers geometry, target depth, the Wi-Fi settings and
# the cloud object ids so a restart reuses the same vector/feature/table.
_persist = {"cloud": {}}
_state_path = None


def default_state_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "monitoring_state.json")


def load_state(path):
    """Read the state file (if any) and apply remembered geometry/target."""
    global _persist, _state_path
    _state_path = path
    try:
        with open(path) as f:
            _persist = json.load(f)
    except (OSError, ValueError):
        _persist = {}
    _persist.setdefault("cloud", {})
    with _lock:
        for key, ck in (("l1", "L1"), ("l2", "L2"), ("target_depth", "target_depth")):
            try:
                if key in _persist:
                    cfg[ck] = float(_persist[key])
            except (TypeError, ValueError):
                pass
    print(f"[state] loaded {path}", flush=True)
    return _persist


def save_state():
    """Persist current geometry/target plus whatever is in _persist (atomically)."""
    with _lock:
        _persist["l1"] = cfg["L1"]
        _persist["l2"] = cfg["L2"]
        _persist["target_depth"] = cfg["target_depth"]
        data = json.dumps(_persist, indent=2)
    try:
        tmp = _state_path + ".tmp"
        with open(tmp, "w") as f:
            f.write(data)
        os.replace(tmp, _state_path)
    except OSError as e:
        print(f"[state] write failed: {e}", flush=True)


def _now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ------------------------------------------------------------------ depth math
def compute_depth(boom_deg, stick_deg):
    """Excavation depth from the two RAW angles, applying sign + zero offset.

    depth = L1*sin(theta_boom) + L2*sin(theta_stick), angles below horizontal
    counting as 'digging down'. Tune signs/offsets in cfg for your mount.
    """
    t1 = cfg["boom_sign"] * (boom_deg - cfg["boom_offset_deg"])
    t2 = cfg["stick_sign"] * (stick_deg - cfg["stick_offset_deg"])
    return cfg["L1"] * math.sin(math.radians(t1)) + \
        cfg["L2"] * math.sin(math.radians(t2))


def build_status():
    """Snapshot in the SAME schema the old ESP32/sensor_receiver served, plus
    the driver name, the (latched) alarms and the cloud/HMI health flags."""
    now = time.monotonic()
    with _lock:
        boom, stick = state["beam"], state["stick"]
        depth = compute_depth(boom["angle_deg"], stick["angle_deg"])
        boom_age = int((now - boom["ts"]) * 1000) if boom["ts"] else -1
        stick_age = int((now - stick["ts"]) * 1000) if stick["ts"] else -1
        stale = cfg["stale_ms"]
        sensor_ok = (boom["mpu_ok"] and stick["mpu_ok"]
                     and 0 <= boom_age < stale and 0 <= stick_age < stale)
        depth_alarm = cfg["target_depth"] > 0 and depth >= cfg["target_depth"]
        loc = {}
        for uid, L in location.items():
            age = int((now - L["ts"]) * 1000) if L["ts"] else -1
            fresh = 0 <= age < cfg["gps_stale_ms"]
            loc[uid] = {
                "gps_ok": bool(L["gps_ok"] and fresh),
                "lat": round(L["lat"], 6), "lon": round(L["lon"], 6),
                "alt_m": round(L["alt_m"], 1), "speed_kph": round(L["speed_kph"], 1),
                "sats": L["sats"], "hacc_m": round(L["hacc_m"], 1),
                "age_ms": age, "seq": L["seq"],
            }
        rt = dict(runtime)
        return {
            "boom_deg": round(boom["angle_deg"], 2),
            "stick_deg": round(stick["angle_deg"], 2),
            "depth_m": round(depth, 3),
            "target_depth": cfg["target_depth"],
            "depth_alarm": depth_alarm,            # raw (no hysteresis)
            "over_dig_alarm": rt["over_dig_alarm"],  # latched by the HMI thread
            "sensor_ok": sensor_ok,
            "sensor_alarm": rt["sensor_alarm"],
            "hse_alarm": rt["hse_alarm"],
            "driver_name": rt["driver_name"],
            "rfid_uid": rt["rfid_uid"],
            "driver_present": rt["driver_present"],
            "cloud_ok": rt["cloud_ok"], "hmi_ok": rt["hmi_ok"],
            "boom_age_ms": boom_age, "stick_age_ms": stick_age,
            "location": loc,
        }


def set_driver(tag, name):
    """Record the active driver (from an RFID scan or POST /rfid).

    A known tag (found in the driver CSV) shows the real name and sets
    driver_present=True; an unknown tag shows "Unknown Driver" with
    driver_present=False. The raw UID is kept in rfid_uid for the cloud feature
    and the session table.
    """
    known = bool(name)
    resolved = name if known else "Unknown Driver"
    with _lock:
        runtime["driver_tag"] = str(tag)
        runtime["rfid_uid"] = str(tag)
        runtime["driver_name"] = resolved
        runtime["driver_present"] = known
    print(f"[rfid] tag={tag} -> driver={resolved} (present={known})", flush=True)


# =============================================================== HTTP ingest ==
app = Flask(__name__)
# Quiet Werkzeug's per-request access log; we print our own parsed line instead.
logging.getLogger("werkzeug").setLevel(logging.ERROR)


@app.post("/ingest")
def ingest():
    """Receive one raw tilt sample from an ESP32 unit."""
    d = request.get_json(silent=True) or {}
    uid = d.get("id")
    if uid not in state:
        return jsonify(error="unknown id (use 'stick' or 'beam')"), 400
    with _lock:
        u = state[uid]
        u["angle_deg"] = float(d.get("angle_deg", u["angle_deg"]))
        u["mpu_ok"] = bool(d.get("mpu_ok", False))
        u["seq"] = int(d.get("seq", -1))
        u["ts"] = time.monotonic()
        angle, mpu_ok, seq = u["angle_deg"], u["mpu_ok"], u["seq"]
    st = build_status()
    print(f"[{uid:>5}] angle={angle:7.2f}  mpu_ok={mpu_ok!s:<5}  seq={seq:<6} | "
          f"depth={st['depth_m']:6.3f}m  alarm={st['over_dig_alarm']!s:<5}  "
          f"sensor_ok={st['sensor_ok']}", flush=True)
    return jsonify(ok=True)


@app.post("/location")
def location_ingest():
    """Receive one GNSS fix from a unit's SIM7000G (stick posts ~every 5 min)."""
    d = request.get_json(silent=True) or {}
    uid = d.get("id")
    if uid not in location:
        return jsonify(error="unknown id (use 'stick' or 'beam')"), 400
    with _lock:
        L = location[uid]
        L["gps_ok"] = bool(d.get("gps_ok", False))
        if L["gps_ok"]:                     # only overwrite coords on a real fix
            L["lat"] = float(d.get("lat", L["lat"]))
            L["lon"] = float(d.get("lon", L["lon"]))
            L["alt_m"] = float(d.get("alt_m", L["alt_m"]))
            L["speed_kph"] = float(d.get("speed_kph", L["speed_kph"]))
            L["sats"] = int(d.get("sats", L["sats"]))
            L["hacc_m"] = float(d.get("hacc_m", L["hacc_m"]))
        L["seq"] = int(d.get("seq", -1))
        L["ts"] = time.monotonic()
        snap = dict(L)
    if snap["gps_ok"]:
        print(f"[{uid:>5}] GPS  lat={snap['lat']:.6f} lon={snap['lon']:.6f}  "
              f"alt={snap['alt_m']:.1f}m  sats={snap['sats']}  "
              f"hacc={snap['hacc_m']:.1f}m  seq={snap['seq']}", flush=True)
    else:
        print(f"[{uid:>5}] GPS  no-fix  seq={snap['seq']}", flush=True)
    return jsonify(ok=True)


@app.get("/status")
def status():
    return jsonify(build_status())


@app.post("/config")
def config():
    """Update target depth / calibration from the HMI or cloud at runtime."""
    d = request.get_json(silent=True) or {}
    with _lock:
        for k in ("L1", "L2", "boom_sign", "stick_sign",
                  "boom_offset_deg", "stick_offset_deg", "target_depth"):
            if k in d:
                cfg[k] = float(d[k])
    save_state()
    return jsonify(ok=True, cfg=cfg)


@app.post("/rfid")
def rfid_inject():
    """Manually inject a tag (USB reader / testing when MFRC522 isn't present)."""
    d = request.get_json(silent=True) or {}
    tag = str(d.get("tag", "")).strip()
    if not tag:
        return jsonify(error="missing 'tag'"), 400
    name = DIRECTORY.lookup(tag) if DIRECTORY else None
    set_driver(tag, name)
    return jsonify(ok=True, tag=tag, driver=runtime["driver_name"])


@app.post("/hse")
def hse_inject():
    """Set the HSE (person) alarm over HTTP.

    Fallback for when the in-process camera thread is disabled or a person
    detector runs on a separate machine: POST {"active": true|false}. The HMI
    thread mirrors it to the panel + buzzer and the cloud thread pushes it.
    """
    d = request.get_json(silent=True) or {}
    active = bool(d.get("active", d.get("alarm", False)))
    with _lock:
        runtime["hse_alarm"] = active
    print(f"[hse] alarm set to {active} via POST /hse", flush=True)
    return jsonify(ok=True, hse_alarm=active)


# ============================================================== DWIN helpers ==
def put_text(dwin, addr, text, pad, ack=False):
    """Write an ASCII string to a TEXT VP, padded/truncated to `pad` bytes."""
    buf = text.encode("ascii", "replace")[:pad].ljust(pad, b"\x20")
    dwin.write_data(addr, buf, ack=ack)


def event_bytes(ev):
    """Raw data bytes carried by an auto-upload frame (handles N-word payloads)."""
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


# ============================================================ HMI controller ==
class HmiController:
    """The ONLY thread that uses the serial port. Writes depth/target/name to the
    panel, services the settings page, and runs the over-dig alarm + buzzer."""

    def __init__(self, dwin, args):
        self.dwin = dwin
        self.args = args
        self.in_settings = False
        self.over_dig = False
        self.hse_shown = False          # last HSE state written to the panel VP
        self.last_push = 0.0
        self.last_beep = 0.0
        # Latest value the panel pushed for each settings VP (seeded from disk).
        self.cache = {
            VP_BEAM_LEN:  str(_persist.get("beam_len_mm", "")),
            VP_STICK_LEN: str(_persist.get("stick_len_mm", "")),
            VP_SSID:      str(_persist.get("wifi_ssid", "")),
            VP_PASSWORD:  str(_persist.get("wifi_password", "")),
        }

    # ----- received auto-upload frames -------------------------------------
    def handle_event(self, ev):
        if ev.cmd != CMD_READ:                       # ignore write ACKs / others
            return
        addr, val = ev.addr, ev.value
        if addr == VP_PAGE_FLAG:                     # settings-page indicator
            if val == TRIGGER and not self.in_settings:
                self.in_settings = True
                print("[hmi] panel entered SETTINGS", flush=True)
            elif val != TRIGGER and self.in_settings:
                self.in_settings = False
            return
        if addr == VP_SAVE_BTN and val == TRIGGER:
            self.dwin.write_single_reg(addr, 0x0000, ack=False)   # consume press
            self._save()
            return
        if addr == VP_CANCEL_BTN and val == TRIGGER:
            self.dwin.write_single_reg(addr, 0x0000, ack=False)
            self._cancel()
            return
        if addr in SETTINGS_VPS:                     # text field auto-uploaded
            text = decode_text(event_bytes(ev))
            self.cache[addr] = text
            shown = text if addr != VP_PASSWORD else "*" * len(text)
            print(f"[hmi] recv 0x{addr:04X} = '{shown}'", flush=True)

    def _save(self):
        beam = self.cache.get(VP_BEAM_LEN, "")
        stick = self.cache.get(VP_STICK_LEN, "")
        ssid = self.cache.get(VP_SSID, "")
        pw = self.cache.get(VP_PASSWORD, "")
        print(f"[hmi] save beam={beam!r}mm stick={stick!r}mm "
              f"ssid={ssid!r} pw={'*' * len(pw)}", flush=True)
        with _lock:
            try:                                     # apply geometry (mm -> m)
                if beam:
                    cfg["L1"] = float(beam) / 1000.0
                if stick:
                    cfg["L2"] = float(stick) / 1000.0
            except ValueError:
                print("[hmi] beam/stick not numeric -- geometry unchanged", flush=True)
            _persist.update(beam_len_mm=beam, stick_len_mm=stick,
                            wifi_ssid=ssid, wifi_password=pw)
        save_state()
        # NOTE: applying the Wi-Fi SSID/password to the OS (wpa_supplicant /
        # NetworkManager) is system-specific -- do it here if you need it.
        self.dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
        self.dwin.goto_page(self.args.main_page, ack=False)
        self.in_settings = False

    def _cancel(self):
        with _lock:                                  # drop edits: reload from disk
            self.cache = {
                VP_BEAM_LEN:  str(_persist.get("beam_len_mm", "")),
                VP_STICK_LEN: str(_persist.get("stick_len_mm", "")),
                VP_SSID:      str(_persist.get("wifi_ssid", "")),
                VP_PASSWORD:  str(_persist.get("wifi_password", "")),
            }
        self.dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
        self.dwin.goto_page(self.args.main_page, ack=False)
        self.in_settings = False

    # ----- writes to the panel ---------------------------------------------
    def _push_values(self, st):
        try:
            put_text(self.dwin, VP_DEPTH_TEXT,   f"{st['depth_m']:.2f}", DEPTH_LEN)
            put_text(self.dwin, VP_TARGET_DEPTH, f"{st['target_depth']:.2f}", TARGET_LEN)
            put_text(self.dwin, VP_DRIVER_NAME,  st["driver_name"], NAME_LEN)
            with _lock:
                runtime["hmi_ok"] = True
        except Exception as e:
            with _lock:
                runtime["hmi_ok"] = False
            print(f"[hmi] push failed: {e}", flush=True)

    def _write_alarm_vp(self, addr, value):
        try:
            self.dwin.write_single_reg(addr, value, ack=False)
        except Exception as e:
            print(f"[alarm] VP 0x{addr:04X} write failed: {e}", flush=True)

    def _evaluate_alarm(self, now, st):
        """Drive both alarms and the buzzer.

        * OVER-DIG: latched with hysteresis, auto-clears when the bucket lifts.
        * HSE (person): set by the camera thread in runtime["hse_alarm"]; the HMI
          just mirrors its edges to the panel VP.
        * BUZZER: beeps while EITHER alarm is active, silent only when BOTH are
          clear (requirement #3).
        """
        depth, target, sensor_ok = st["depth_m"], st["target_depth"], st["sensor_ok"]

        # --- over-dig latch with hysteresis ---
        over = self.over_dig
        if not over and target > 0 and depth >= target:
            over = True
            self._write_alarm_vp(self.args.overdig_vp, 1)
            print(f"[alarm] OVER-DIG  depth={depth:.3f} >= target={target:.3f}", flush=True)
        elif over and depth < target - self.args.hysteresis:
            over = False
            self._write_alarm_vp(self.args.overdig_vp, 0)
            print(f"[alarm] over-dig cleared  depth={depth:.3f} < "
                  f"target-{self.args.hysteresis}", flush=True)
        self.over_dig = over

        # --- HSE (person) alarm: mirror the camera thread's flag to the panel ---
        hse = bool(st["hse_alarm"])
        if hse != self.hse_shown:
            self._write_alarm_vp(self.args.hse_vp, 1 if hse else 0)
            self.hse_shown = hse
            print(f"[alarm] HSE {'RAISED (person detected)' if hse else 'cleared'}",
                  flush=True)

        with _lock:
            runtime["over_dig_alarm"] = over
            runtime["sensor_alarm"] = not sensor_ok

        # --- buzzer: beep while EITHER alarm is up, to push the operator ---
        if (over or hse) and now - self.last_beep >= self.args.beep_period:
            self.last_beep = now
            try:
                self.dwin.buzzer(BuzzerDuration.BUZZ_500MSEC, ack=False)
            except Exception:
                pass

    def run(self):
        print("[hmi] servicing panel (only this thread owns the serial port)",
              flush=True)
        while not _stop.is_set():
            try:
                ev = self.dwin.read_event(timeout=0.05)
                if ev is not None:
                    self.handle_event(ev)
            except Exception as e:
                print(f"[hmi] read error: {e}", flush=True)
            now = time.monotonic()
            st = build_status()
            if now - self.last_push >= self.args.push_interval:
                self.last_push = now
                self._push_values(st)
            self._evaluate_alarm(now, st)
        # leave the panel in a safe state on shutdown
        self._write_alarm_vp(self.args.overdig_vp, 0)
        self._write_alarm_vp(self.args.hse_vp, 0)


# ================================================================ cloud sync ==
class CloudSync:
    """GEOMind (geobox SDK) sync: per-device vector layer + one feature + a table.

    Feature fields (attributes) and table columns. Alarms are stored as Integer
    0/1 because the SDK's FieldType has no Boolean member.
    """
    FEATURE_FIELDS = [
        ("target_depth", "Float"), ("current_depth", "Float"), ("driver", "String"),
        ("rfid_uid", "String"), ("driver_present", "Integer"),
        ("over_dig_alarm", "Integer"), ("sensor_alarm", "Integer"),
        ("hse_alarm", "Integer"), ("updated_at", "String"),
    ]
    TABLE_FIELDS = [
        ("ts", "String"), ("driver", "String"), ("depth", "Float"),
        ("target_depth", "Float"), ("over_dig_alarm", "Integer"),
        ("sensor_alarm", "Integer"), ("hse_alarm", "Integer"),
        ("lat", "Float"), ("lon", "Float"), ("event", "String"),
    ]
    # Driver Operation History (requirement #6): one row per driver session,
    # its alarm counters updated live during the session.
    SESSION_FIELDS = [
        ("driver", "String"), ("rfid_uid", "String"), ("date", "String"),
        ("start_time", "String"), ("end_time", "String"),
        ("total_alarms", "Integer"), ("hse_alarm_count", "Integer"),
        ("over_dig_alarm_count", "Integer"),
    ]

    def __init__(self, args, directory):
        self.args = args
        self.directory = directory
        self.client = None
        self.layer = None
        self.feature_id = None
        self.table = None
        self.last_logged = None        # (over, sensor, hse, tag) of the last row
        self.last_heartbeat = 0.0
        # --- driver-session state (requirement #6) ---
        self.session_table = None
        self.session_row = None        # TableRow of the OPEN session (or None)
        self.session_key = None        # driver_tag of the open session
        self.session_counts = {"total_alarms": 0, "hse_alarm_count": 0,
                               "over_dig_alarm_count": 0}
        self.session_edges = {"over_dig_alarm": False, "hse_alarm": False}

    @staticmethod
    def _ft(name):
        return getattr(FieldType, name)   # "Float" -> FieldType.Float, etc.

    def _ensure_fields(self, obj, fields):
        """Add any missing fields to a layer/table (idempotent).

        add_field on a field that already exists raises; we swallow that so an
        object created by an older version gains the new columns on reuse.
        """
        for n, t in fields:
            try:
                obj.add_field(name=n, data_type=self._ft(t))
            except Exception:
                pass   # already exists (or transient) -- safe to ignore

    def connect(self):
        self.client = GeoboxClient(host=self.args.geomind_host,
                                   username=self.args.geomind_user,
                                   password=self.args.geomind_pass,
                                   verify=not self.args.insecure)
        print(f"[cloud] authenticated to {self.args.geomind_host} "
              f"as {self.args.geomind_user}", flush=True)

    # ----- first-run create / later-run reuse ------------------------------
    def ensure_objects(self):
        dev = self.args.device_id
        vname, tname = f"{dev}_status", f"{dev}_log"
        sname = f"{dev}_sessions"
        cloud = _persist.get("cloud", {})

        # ---- vector layer (the "feature"/"vector") ----
        layer = None
        if cloud.get("vector_uuid"):
            try:
                layer = self.client.get_vector(cloud["vector_uuid"])
            except Exception:
                layer = None
        if layer is None:
            layer = self.client.get_vector_by_name(vname)
        if layer is None:
            print(f"[cloud] creating vector layer '{vname}'", flush=True)
            layer = self.client.create_vector(name=vname, layer_type=LayerType.Point,
                                              display_name=f"{dev} status")
        self.layer = layer
        # Add any missing fields -- covers a fresh layer AND a layer created by an
        # older version that lacks rfid_uid / driver_present.
        self._ensure_fields(layer, self.FEATURE_FIELDS)

        # ---- the single status feature ----
        feature = None
        if cloud.get("feature_id") is not None:
            try:
                feature = layer.get_feature(cloud["feature_id"], out_srid=4326)
            except Exception:
                feature = None
        if feature is None:
            feats = layer.get_features(limit=1, out_srid=4326)
            if feats:
                feature = feats[0]
        if feature is None:
            print("[cloud] creating status feature", flush=True)
            with _lock:
                tgt, name = cfg["target_depth"], runtime["driver_name"]
            # Start the marker at the configured site (near Muscat) so the map
            # shows the equipment somewhere real until a live GNSS fix arrives.
            lat, lon = self.args.home_lat, self.args.home_lon
            feature = layer.create_feature(geojson={
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {"target_depth": tgt, "current_depth": 0.0,
                               "driver": name, "rfid_uid": "", "driver_present": 0,
                               "over_dig_alarm": 0, "sensor_alarm": 0,
                               "hse_alarm": 0, "updated_at": _now_iso()},
            }, srid=4326)
        try:
            self.feature_id = feature.id
        except AttributeError:                       # be defensive about create()
            feats = layer.get_features(limit=1, order_by="id D", out_srid=4326)
            self.feature_id = feats[0].id if feats else None

        # ---- the dig-log table ----
        table = None
        if cloud.get("table_uuid"):
            try:
                table = self.client.get_table(cloud["table_uuid"])
            except Exception:
                table = None
        if table is None:
            table = self.client.get_table_by_name(tname)
        if table is None:
            print(f"[cloud] creating table '{tname}'", flush=True)
            table = self.client.create_table(name=tname, display_name=f"{dev} dig log")
        self.table = table
        self._ensure_fields(table, self.TABLE_FIELDS)

        # ---- the driver-session history table (requirement #6) ----
        session_table = None
        if cloud.get("session_table_uuid"):
            try:
                session_table = self.client.get_table(cloud["session_table_uuid"])
            except Exception:
                session_table = None
        if session_table is None:
            session_table = self.client.get_table_by_name(sname)
        if session_table is None:
            print(f"[cloud] creating session table '{sname}'", flush=True)
            session_table = self.client.create_table(
                name=sname, display_name=f"{dev} driver sessions")
        self.session_table = session_table
        self._ensure_fields(session_table, self.SESSION_FIELDS)

        # ---- persist the ids/names so a restart reuses them ----
        with _lock:
            _persist.setdefault("cloud", {}).update(
                device_id=dev, vector_name=vname, vector_uuid=layer.uuid,
                feature_id=self.feature_id, table_name=tname, table_uuid=table.uuid,
                session_table_name=sname, session_table_uuid=session_table.uuid)
        save_state()
        print(f"[cloud] ready: vector={layer.uuid} feature={self.feature_id} "
              f"table={table.uuid} sessions={session_table.uuid}", flush=True)

    # ----- one sync cycle ---------------------------------------------------
    def poll_and_push(self):
        # 1) read the target depth the operator set in the GEOMind web UI
        feature = self.layer.get_feature(self.feature_id, out_srid=4326)
        props = feature.data.setdefault("properties", {})
        tgt = props.get("target_depth")
        if tgt is not None:
            try:
                tv = float(tgt)
                with _lock:
                    changed = cfg["target_depth"] != tv
                    cfg["target_depth"] = tv
                if changed:
                    print(f"[cloud] target_depth <- {tv} m (from cloud)", flush=True)
                    save_state()
            except (TypeError, ValueError):
                pass

        # 2) push current depth + all alarms + driver + location into the feature
        st = build_status()
        with _lock:
            rt = dict(runtime)
            target = cfg["target_depth"]
        # Equipment location: prefer a fresh GNSS fix from the stick, else fall
        # back to the configured site near Muscat so the marker stays on the map.
        stick_loc = st["location"]["stick"]
        if stick_loc["gps_ok"] and (stick_loc["lat"] or stick_loc["lon"]):
            lat, lon = stick_loc["lat"], stick_loc["lon"]
        else:
            lat, lon = self.args.home_lat, self.args.home_lon
        props.update(current_depth=st["depth_m"], target_depth=target,
                     driver=rt["driver_name"], rfid_uid=rt["rfid_uid"],
                     driver_present=int(rt["driver_present"]),
                     over_dig_alarm=int(rt["over_dig_alarm"]),
                     sensor_alarm=int(rt["sensor_alarm"]),
                     hse_alarm=int(rt["hse_alarm"]), updated_at=_now_iso())
        geom = feature.data.setdefault("geometry", {"type": "Point", "coordinates": [0, 0]})
        geom["coordinates"] = [lon, lat]
        feature.save()

        # 3) driver-session history: open/close sessions + count alarm edges
        self._update_session(rt)

        # 4) append a heartbeat/edge row to the dig-log table
        self._maybe_log(st, rt, lat, lon, target)

    def _maybe_log(self, st, rt, lat, lon, target):
        key = (bool(rt["over_dig_alarm"]), bool(rt["sensor_alarm"]),
               bool(rt["hse_alarm"]), rt["driver_tag"])
        now = time.monotonic()
        first = self.last_logged is None
        changed = (not first) and key != self.last_logged
        due = (now - self.last_heartbeat) >= self.args.row_interval
        if not (first or changed or due):
            return
        event = self._event_label(key, first, changed)
        try:
            self.table.create_row(
                ts=_now_iso(), driver=rt["driver_name"], depth=st["depth_m"],
                target_depth=target, over_dig_alarm=int(rt["over_dig_alarm"]),
                sensor_alarm=int(rt["sensor_alarm"]), hse_alarm=int(rt["hse_alarm"]),
                lat=lat, lon=lon, event=event)
            print(f"[cloud] row '{event}' logged (depth={st['depth_m']:.3f}m)", flush=True)
        except Exception as e:
            print(f"[cloud] row log failed: {e}", flush=True)
            return
        self.last_logged = key
        self.last_heartbeat = now      # reset heartbeat after any row write

    def _event_label(self, key, first, changed):
        if first:
            return "init"
        if not changed:
            return "heartbeat"
        prev, labels = self.last_logged, []
        over, sensor, hse, tag = key
        if prev[0] != over:
            labels.append("over_dig_on" if over else "over_dig_off")
        if prev[1] != sensor:
            labels.append("sensor_alarm_on" if sensor else "sensor_alarm_off")
        if prev[2] != hse:
            labels.append("hse_on" if hse else "hse_off")
        if prev[3] != tag:
            labels.append("driver_change")
        return ",".join(labels) if labels else "heartbeat"

    # ----- driver-session history (requirement #6) --------------------------
    def _update_session(self, rt):
        """Open a session row on each driver change, close the previous one, and
        bump the alarm counters on every alarm rising edge during the session."""
        if self.session_table is None:
            return
        tag = rt["driver_tag"]
        if tag and tag != self.session_key:      # a (new) driver signed in
            self._close_session()                # stamp end_time on the old row
            self._open_session(rt)
            self.session_key = tag
        if self.session_row is not None:
            self._count_edge(rt, "over_dig_alarm", "over_dig_alarm_count")
            self._count_edge(rt, "hse_alarm", "hse_alarm_count")

    def _open_session(self, rt):
        now = datetime.now()                     # local wall-clock, for humans
        self.session_counts = {"total_alarms": 0, "hse_alarm_count": 0,
                               "over_dig_alarm_count": 0}
        # Seed edges with the CURRENT alarm states so an alarm already active at
        # sign-in is NOT miscounted as a fresh event for this session.
        self.session_edges = {"over_dig_alarm": bool(rt["over_dig_alarm"]),
                              "hse_alarm": bool(rt["hse_alarm"])}
        try:
            self.session_row = self.session_table.create_row(
                driver=rt["driver_name"], rfid_uid=rt["rfid_uid"],
                date=now.strftime("%Y-%m-%d"),
                start_time=now.strftime("%H:%M:%S"), end_time="",
                total_alarms=0, hse_alarm_count=0, over_dig_alarm_count=0)
            print(f"[cloud] session opened: {rt['driver_name']} "
                  f"(uid={rt['rfid_uid']})", flush=True)
        except Exception as e:
            self.session_row = None
            print(f"[cloud] session open failed: {e}", flush=True)

    def _count_edge(self, rt, flag, count_col):
        active = bool(rt[flag])
        if active and not self.session_edges.get(flag, False):   # rising edge
            self.session_counts[count_col] += 1
            self.session_counts["total_alarms"] += 1
            self._push_session_counts()
        self.session_edges[flag] = active

    def _push_session_counts(self):
        if self.session_row is None:
            return
        try:
            self.session_row.update(**self.session_counts)
        except Exception as e:
            print(f"[cloud] session counter update failed: {e}", flush=True)

    def _close_session(self):
        if self.session_row is None:
            return
        end = datetime.now().strftime("%H:%M:%S")
        try:
            self.session_row.update(end_time=end, **self.session_counts)
            print(f"[cloud] session closed (end {end}, "
                  f"alarms={self.session_counts['total_alarms']})", flush=True)
        except Exception as e:
            print(f"[cloud] session close failed: {e}", flush=True)
        self.session_row = None

    # ----- thread body ------------------------------------------------------
    def run(self):
        while not _stop.is_set():                    # connect + create/reuse
            try:
                self.connect()
                self.ensure_objects()
                try:
                    self.directory.refresh(self.client)
                except Exception as e:
                    print(f"[cloud] CSV load failed: {e}", flush=True)
                break
            except Exception as e:
                print(f"[cloud] setup failed ({e}); retry in 10s", flush=True)
                if _stop.wait(10):
                    return
        last_csv = time.monotonic()
        while not _stop.is_set():
            try:
                self.poll_and_push()
                with _lock:
                    runtime["cloud_ok"] = True
            except Exception as e:
                with _lock:
                    runtime["cloud_ok"] = False
                print(f"[cloud] sync error: {e}", flush=True)
            now = time.monotonic()
            # Refresh the driver CSV on a timer OR immediately when the RFID thread
            # flags an unknown tag (requirement #1: "download + search the CSV").
            on_demand = _rfid_refresh.is_set()
            if on_demand or now - last_csv >= self.args.csv_refresh:
                last_csv = now
                _rfid_refresh.clear()
                try:
                    self.directory.refresh(self.client)
                    if on_demand:
                        print("[cloud] driver CSV re-downloaded (unknown tag)",
                              flush=True)
                except Exception as e:
                    print(f"[cloud] CSV refresh failed: {e}", flush=True)
            if _stop.wait(self.args.cloud_interval):
                break
        # stamp the end time on any still-open session at shutdown
        self._close_session()


# ========================================================== driver directory ==
class DriverDirectory:
    """RFID tag -> driver name, loaded from a CSV on the cloud (or a local file).

    Columns are auto-detected: a tag column (tag/uid/rfid/card/id) and a name
    column (name/driver/operator), falling back to the first two columns.
    """
    TAG_COLS = ("tag", "uid", "rfid", "card", "id", "tag_id", "cardid")
    NAME_COLS = ("name", "driver", "driver_name", "operator", "fullname", "full_name")

    def __init__(self, args):
        self.args = args
        self.map = {}
        self._lock = threading.Lock()

    @staticmethod
    def _norm(tag):
        return str(tag).strip().lower().replace(":", "").replace(" ", "")

    def _parse(self, text):
        reader = csv.DictReader(io.StringIO(text))
        cols = reader.fieldnames or []
        if not cols:
            return {}
        low = {c.lower().strip(): c for c in cols}
        tag_col = next((low[c] for c in self.TAG_COLS if c in low), cols[0])
        name_col = next((low[c] for c in self.NAME_COLS if c in low),
                        cols[1] if len(cols) > 1 else cols[0])
        out = {}
        for row in reader:
            tag = (row.get(tag_col) or "").strip()
            name = (row.get(name_col) or "").strip()
            if tag:
                out[self._norm(tag)] = name
        return out

    def refresh(self, client=None):
        """(Re)load the directory. Prefers the cloud CSV, falls back to local."""
        text = None
        if client and self.args.driver_csv:
            files = client.get_files_by_name(self.args.driver_csv)
            if files:
                # in-memory read (no disk, no tqdm) via the same download endpoint
                resp = client.get(f"{files[0].endpoint}download/", stream=True)
                text = resp.text
            else:
                print(f"[rfid] cloud CSV '{self.args.driver_csv}' not found", flush=True)
        if text is None and self.args.driver_csv_local:
            try:
                with open(self.args.driver_csv_local, encoding="utf-8") as fh:
                    text = fh.read()
            except OSError as e:
                print(f"[rfid] local CSV read failed: {e}", flush=True)
        if text is None:
            return
        new = self._parse(text)
        with self._lock:
            self.map = new
        print(f"[rfid] driver directory loaded: {len(new)} tags", flush=True)

    def lookup(self, tag):
        with self._lock:
            return self.map.get(self._norm(tag))


# ================================================================ RFID reader ==
class RfidReader:
    """MFRC522 (SPI) reader thread. Pi-only; degrades to POST /rfid elsewhere."""

    def __init__(self, args, directory):
        self.args = args
        self.directory = directory
        self.reader = None
        self.last_uid = None
        self.last_seen = 0.0

    def _candidates(self, uid):
        """The CSV tag may be the decimal UID or a hex form -- try them all."""
        cands = [str(uid)]
        try:
            cands += [format(int(uid), "x"), format(int(uid), "X")]
        except (TypeError, ValueError):
            pass
        return cands

    def _handle(self, uid):
        name = None
        for cand in self._candidates(uid):
            name = self.directory.lookup(cand)
            if name:
                break
        print(f"[rfid] scanned UID={uid}", flush=True)   # log so the CSV can be filled
        set_driver(uid, name)
        if name is None:
            # Unknown tag: ask the cloud thread to re-download the driver CSV so a
            # just-added driver is picked up on a re-scan (non-blocking here).
            _rfid_refresh.set()

    def run(self):
        if not _MFRC522_OK:
            print(f"[rfid] MFRC522 unavailable ({_MFRC522_ERR}); "
                  f"use POST /rfid to inject tags", flush=True)
            return
        try:
            self.reader = SimpleMFRC522()
        except Exception as e:
            print(f"[rfid] reader init failed ({e}); use POST /rfid", flush=True)
            return
        print("[rfid] MFRC522 ready -- scan a card", flush=True)
        try:
            while not _stop.is_set():
                try:
                    uid, _text = self.reader.read_no_block()
                except Exception as e:
                    print(f"[rfid] read error: {e}", flush=True)
                    uid = None
                now = time.monotonic()
                if uid is not None and not (uid == self.last_uid
                                            and now - self.last_seen < 2.0):
                    self.last_uid, self.last_seen = uid, now
                    self._handle(uid)
                elif uid is not None:
                    self.last_seen = now                 # refresh debounce window
                if _stop.wait(0.2):
                    break
        finally:
            try:
                import RPi.GPIO as GPIO
                GPIO.cleanup()
            except Exception:
                pass


# ============================================================ camera / HSE ====
class CameraDetector:
    """Runs the YOLOv4-tiny person-in-zone detector (person_detector.py) on a
    background thread and mirrors its alarm EDGES into runtime["hse_alarm"].

    Like the cloud/RFID threads it never touches the serial port -- the HMI
    thread reads runtime["hse_alarm"] and drives the panel VP + buzzer. If
    OpenCV is missing or the RTSP stream can't be opened it logs and exits the
    thread; the rest of the system keeps running and POST /hse still works.
    """

    def __init__(self, args):
        self.args = args

    def _on_change(self, active):
        """Called by the detector on each alarm edge (True raised / False clear)."""
        with _lock:
            runtime["hse_alarm"] = bool(active)
        print(f"[camera] HSE alarm {'RAISED' if active else 'cleared'}", flush=True)

    @staticmethod
    def _log(msg):
        print(f"[camera] {msg}", flush=True)

    def run(self):
        if not _CV2_OK:
            print(f"[camera] OpenCV unavailable ({_CV2_ERR}); person detection "
                  f"disabled -- use POST /hse to set the HSE alarm", flush=True)
            return
        try:
            from person_detector import PersonZoneMonitor, build_rtsp_url, parse_zone
        except Exception as e:
            print(f"[camera] person_detector import failed ({e}); disabled", flush=True)
            return
        try:
            zone = parse_zone(self.args.zone)
        except Exception as e:
            print(f"[camera] bad --zone ({e}); watching the whole frame", flush=True)
            zone = (0.0, 0.0, 1.0, 1.0)
        url = build_rtsp_url(self.args.camera_ip, self.args.camera_code,
                             sub=not self.args.camera_main)
        print(f"[camera] opening {url.replace(self.args.camera_code, '******')}",
              flush=True)
        monitor = PersonZoneMonitor(
            url, zone=zone, dwell=self.args.dwell, conf=self.args.conf,
            nms=self.args.nms, overlap=self.args.overlap, grace=self.args.grace,
            input_size=self.args.yolo_input, models_dir=self.args.models_dir,
            on_change=self._on_change, on_log=self._log,
            save_dir=self.args.camera_save_dir)
        # Restart the whole detector loop if the stream dies, until shutdown.
        while not _stop.is_set():
            try:
                monitor.run(_stop)
                break                          # clean stop (stop_event was set)
            except Exception as e:
                with _lock:
                    runtime["hse_alarm"] = False   # fail safe: never leave it stuck on
                print(f"[camera] detector error ({e}); retry in 10s", flush=True)
                if _stop.wait(10):
                    break


# ===================================================================== main ===
def main():
    ap = argparse.ArgumentParser(
        description="Integrated excavation monitor: ESP32 ingest + DWIN HMI + "
                    "GEOMind cloud + RFID driver id")
    # HTTP (ESP32 ingest server)
    ap.add_argument("--http-host", default="192.168.100.43")
    ap.add_argument("--http-port", type=int, default=5000)
    # DWIN serial panel
    ap.add_argument("--dwin-port", default="/dev/serial0")
    ap.add_argument("--dwin-baud", type=int, default=115200)
    ap.add_argument("--debug", action="store_true", help="dump every DWIN TX/RX frame")
    ap.add_argument("--main-page", type=int, default=0, help="main screen page index")
    # geometry / depth
    ap.add_argument("--l1", type=float, default=cfg["L1"], help="boom length (m)")
    ap.add_argument("--l2", type=float, default=cfg["L2"], help="stick length (m)")
    ap.add_argument("--target", type=float, default=cfg["target_depth"],
                    help="initial target depth (m); the cloud overrides it at runtime")
    ap.add_argument("--stale-ms", type=int, default=cfg["stale_ms"])
    ap.add_argument("--gps-stale-ms", type=int, default=cfg["gps_stale_ms"])
    # HMI / alarm
    ap.add_argument("--push-interval", type=float, default=1.0,
                    help="seconds between depth/target/name writes to the panel")
    ap.add_argument("--hysteresis", type=float, default=0.05,
                    help="m below target the depth must fall to clear the alarm")
    ap.add_argument("--beep-period", type=float, default=1.0,
                    help="seconds between buzzer beeps while over-digging")
    # cloud (GEOMind)
    ap.add_argument("--geomind-host", default="https://app.geo-mind.ai")
    ap.add_argument("--geomind-user", default="pdo.excavator")
    ap.add_argument("--geomind-pass", default="PDO@excavator1")
    ap.add_argument("--device-id", default="excavator1",
                    help="names the per-device cloud objects: <id>_status, <id>_log")
    ap.add_argument("--cloud-interval", type=float, default=5.0,
                    help="seconds between cloud read-target/push-feature cycles")
    ap.add_argument("--row-interval", type=float, default=10.0,
                    help="seconds between heartbeat table rows")
    ap.add_argument("--insecure", action="store_true", help="skip TLS verification")
    ap.add_argument("--no-cloud", action="store_true", help="disable cloud sync")
    # RFID / driver CSV
    ap.add_argument("--driver-csv", default="Driver_name_database.csv",
                    help="name of the tag->driver CSV stored on the cloud")
    ap.add_argument("--driver-csv-local", default=None,
                    help="fallback local CSV path if the cloud CSV is unreachable")
    ap.add_argument("--csv-refresh", type=float, default=300.0,
                    help="seconds between driver-directory refreshes")
    ap.add_argument("--no-rfid", action="store_true", help="disable the RFID thread")
    # HMI alarm VPs (panel-side data variables; add matching VPs in the DGUS project)
    ap.add_argument("--overdig-vp", type=lambda x: int(x, 0), default=VP_OVERDIG_ALARM,
                    help="VP for the over-dig alarm flag (default 0x0401)")
    ap.add_argument("--hse-vp", type=lambda x: int(x, 0), default=VP_HSE_ALARM,
                    help="VP for the HSE/person alarm flag (default 0x0402)")
    # equipment home location -- the map marker sits here until a live GNSS fix
    ap.add_argument("--home-lat", type=float, default=23.5900,
                    help="fallback latitude for the map marker (default: Muscat, Oman)")
    ap.add_argument("--home-lon", type=float, default=58.4059,
                    help="fallback longitude for the map marker (default: Muscat, Oman)")
    # camera / person detection -> HSE alarm
    ap.add_argument("--camera-ip", default="192.168.100.13", help="EZVIZ camera IP")
    ap.add_argument("--camera-code", default=os.environ.get("EZVIZ_CODE", "NANXJW"),
                    help="camera RTSP verification code (or set EZVIZ_CODE)")
    ap.add_argument("--camera-main", action="store_true",
                    help="use the main (HD) stream; default sub (lighter on the Pi)")
    ap.add_argument("--zone", default="0,0,1,1",
                    help="person watch-zone x1,y1,x2,y2 as fractions 0..1")
    ap.add_argument("--dwell", type=float, default=3.0,
                    help="seconds a person must stay in zone before the HSE alarm")
    ap.add_argument("--conf", type=float, default=0.50, help="YOLO confidence 0..1")
    ap.add_argument("--nms", type=float, default=0.40, help="YOLO NMS IoU threshold")
    ap.add_argument("--overlap", type=float, default=0.30,
                    help="min box-in-zone overlap fraction to count as inside")
    ap.add_argument("--grace", type=float, default=1.0,
                    help="seconds of absence tolerated before the dwell timer resets")
    ap.add_argument("--yolo-input", type=int, default=416,
                    help="YOLO input size (320 faster, 416 default, 608 more accurate)")
    ap.add_argument("--models-dir",
                    default=os.path.join(os.path.dirname(os.path.dirname(
                        os.path.abspath(__file__))), "ezviz_camera", "models"),
                    help="directory holding yolov4-tiny.cfg/.weights")
    ap.add_argument("--camera-save-dir", default=None,
                    help="if set, save an annotated JPG when the HSE alarm fires")
    ap.add_argument("--no-camera", action="store_true",
                    help="disable the in-process person detector (POST /hse still works)")
    # state file
    ap.add_argument("--state-file", default=default_state_path())
    args = ap.parse_args()

    # CLI defaults into cfg, then overlay the persisted state (last cloud target,
    # geometry, etc. survive a restart).
    cfg.update(L1=args.l1, L2=args.l2, target_depth=args.target,
               stale_ms=args.stale_ms, gps_stale_ms=args.gps_stale_ms)
    load_state(args.state_file)

    global DIRECTORY
    DIRECTORY = DriverDirectory(args)
    if args.driver_csv_local:               # cloud refresh happens in the cloud thread
        try:
            DIRECTORY.refresh()
        except Exception:
            pass

    # serial / HMI (the only serial owner)
    dwin = DwinLCD(args.dwin_port, args.dwin_baud, debug=args.debug)
    if not dwin.is_connected():
        print("[warn] DWIN not responding -- check wiring / baud / UART setup", flush=True)
    hmi = HmiController(dwin, args)

    threads = [threading.Thread(target=hmi.run, name="hmi", daemon=True)]
    if args.no_cloud:
        print("[main] cloud sync disabled (--no-cloud)", flush=True)
    elif not _GEOBOX_OK:
        print(f"[warn] geobox not importable ({_GEOBOX_ERR}); cloud disabled", flush=True)
    else:
        threads.append(threading.Thread(target=CloudSync(args, DIRECTORY).run,
                                        name="cloud", daemon=True))
    if args.no_rfid:
        print("[main] RFID disabled (--no-rfid); POST /rfid still works", flush=True)
    else:
        threads.append(threading.Thread(target=RfidReader(args, DIRECTORY).run,
                                        name="rfid", daemon=True))
    if args.no_camera:
        print("[main] camera disabled (--no-camera); POST /hse still works", flush=True)
    elif not _CV2_OK:
        print(f"[warn] OpenCV not importable ({_CV2_ERR}); camera disabled", flush=True)
    else:
        threads.append(threading.Thread(target=CameraDetector(args).run,
                                        name="camera", daemon=True))

    for t in threads:
        t.start()

    print(f"[main] HTTP on {args.http_host}:{args.http_port}  "
          f"depth=L1*sin(boom)+L2*sin(stick)  L1={cfg['L1']} L2={cfg['L2']} "
          f"target={cfg['target_depth']}m  (POST /ingest /location /rfid, "
          f"GET /status, POST /config)", flush=True)
    try:
        # threaded=True so two ESP32s + /status pollers don't block each other.
        app.run(host=args.http_host, port=args.http_port, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        _stop.set()
        time.sleep(0.3)
        try:
            dwin.close()
        except Exception:
            pass
        print("\n[main] stopped", flush=True)


if __name__ == "__main__":
    main()
