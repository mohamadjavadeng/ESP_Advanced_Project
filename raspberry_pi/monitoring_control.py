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
     depth (0x0300) to the panel and drives the ALARMS: OVER-DIG (VP 0x0401)
     when depth >= target, HSE/person (VP 0x0400) from the camera thread, beeping
     the buzzer whenever EITHER is active (silent only when both clear). Over-dig
     auto-clears once the operator lifts the bucket (depth < target - hyst). A
     main-page TARE key (VP 0x1000 == 0x00FF) zeroes the depth at the current
     position. On the SETTINGS page beam (0x0012) / stick (0x2000) / wifi (ssid
     0x0330, pw 0x0350) auto-upload with a 1 s confirm beep; beam/stick apply
     LIVE but only persist on SAVE (0x0011), while BACK (0x0010) reverts every
     field to the last saved value.

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

  5. CAMERA HSE ALARM (TCP link) -- person detection runs as the separate,
     field-proven script ezviz_camera/rpi_person_zone_alarm.py (it owns the
     camera + YOLO model). That script connects to THIS program over a TCP
     socket (--hse-port 5050, newline-delimited JSON) and sends the alarm
     state plus the filename of the saved evidence picture. This program sets
     runtime["hse_alarm"] (panel VP 0x0400 + buzzer + cloud) and the cloud
     thread uploads the alarm JPG and ATTACHES it to the status feature.
     POST /hse stays as a manual fallback. Fail-safe: a camera-client
     disconnect or 30 s of silence clears the alarm.

  6. DRIVER SESSIONS -- the cloud thread opens a row in a <device>_sessions table
     on each driver change, counts HSE / over-dig alarm edges during the session,
     and stamps the end time when the next driver signs in (or on shutdown). The
     open session survives a restart (its row id + counts are persisted).

The cloud and RFID threads NEVER touch the serial port -- they only update
shared state; the HMI thread reads that state and writes the panel, exactly like
dwin_hmi_app.py, so nothing fights for the serial lock. The excavator keeps
working (depth, alarm, buzzer) even with NO network -- cloud/RFID failures are
caught and retried, and the last target depth is persisted to the state file so
the alarm still has a threshold after an offline restart.

Run from the raspberry_pi/ folder (so `import dwin_lcd` resolves):
    pip install flask pyserial geobox tqdm mfrc522 openpyxl
    export GEOMIND_APIKEY=...            # the pdo_device_service key (ask Hamed)
    python3 monitoring_control.py \
        --http-port 5000 --dwin-port /dev/serial0 --dwin-baud 115200 \
        --geomind-host https://app.geo-mind.ai \
        --device-id excavator1 --driver-csv Driver_name_database.xlsx

Auth uses the session-less API KEY ($GEOMIND_APIKEY or --geomind-apikey), which
does not evict the human portal user and is never revoked mid-run. Only if no key
is set does it fall back to a password login (--geomind-user/--geomind-pass or
$GEOMIND_PASS). Keep the key out of git and logs.

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
import queue
import socket
import threading
import time
from datetime import datetime, timezone

from flask import Flask, jsonify, request

from dwin_lcd import DwinLCD, BuzzerDuration

# --- optional deps: the program runs without them; the matching subsystem just
#     degrades gracefully (so this file is also testable on a dev PC). ---------
class _NoAuthErr(Exception):     # sentinel: never raised; used only as the
    pass                         # AuthenticationError fallback on older SDKs

try:
    from geobox import GeoboxClient
    from geobox.enums import LayerType, FieldType
    _GEOBOX_OK, _GEOBOX_ERR = True, None
    try:                         # 401 (revoked/invalid auth) surfaces as this
        from geobox.exception import AuthenticationError
    except Exception:            # SDK too old to expose it -> typed catch is a no-op
        AuthenticationError = _NoAuthErr
except Exception as _e:                      # ImportError or any load error
    GeoboxClient = LayerType = FieldType = None
    AuthenticationError = _NoAuthErr
    _GEOBOX_OK, _GEOBOX_ERR = False, _e

try:
    from mfrc522 import SimpleMFRC522         # pulls in RPi.GPIO + spidev (Pi only)
    _MFRC522_OK, _MFRC522_ERR = True, None
except Exception as _e:
    SimpleMFRC522 = None
    _MFRC522_OK, _MFRC522_ERR = False, _e

# --------------------------------------------------------------- VP address map
# Page 1 (Pi -> panel)
VP_DEPTH_TEXT    = 0x0001   # current depth, written as TEXT
VP_DRIVER_NAME   = 0x0200   # driver name, TEXT, max 20 chars
VP_TARGET_DEPTH  = 0x0300   # target-depth field (from the cloud)
VP_OVERDIG_ALARM = 0x0401   # over-dig alarm flag: 1 = alarm, 0 = clear
VP_HSE_ALARM     = 0x0400   # HSE/person alarm flag: 1 = alarm, 0 = clear (--hse-vp)

# Main-page "tare depth" key: the panel auto-uploads 0x1000 == 0x00FF when the
# operator presses it -> capture the current depth as the zero offset.
VP_DEPTH_OFFSET = 0x1000
OFFSET_TRIGGER  = 0x00FF

# Page 2 (panel -> Pi, auto-upload)
VP_BEAM_LEN  = 0x0012      # beam length, TEXT (mm)
VP_STICK_LEN = 0x2000      # stick length, TEXT (mm) -- 0x2000 in the CURRENT DGUS
                           # project (operator-confirmed 2026-07-06; older compiled
                           # bins used 0x0016). If stick edits are ignored, re-check
                           # this VP + its data-auto-upload flag in the panel.
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
# Settings-field widths for prefill / BACK write-back. MUST equal the text length
# set for each VP in the DGUS project, or a write overruns into the next VP.
LEN_TEXT_LEN, SSID_LEN, PASS_LEN = 8, 20, 20


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
    "depth_offset": 0.0,             # zero/tare offset set from the HMI 0x1000 key
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
    "hse_alarm": False,              # set by the camera script via the TCP link
    "hse_picture": "",               # file name of the last alarm evidence JPG
    "camera_status": "off",          # live camera-link state (GET /status)
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
        for key, ck in (("l1", "L1"), ("l2", "L2"), ("target_depth", "target_depth"),
                        ("depth_offset", "depth_offset")):
            try:
                if key in _persist:
                    cfg[ck] = float(_persist[key])
            except (TypeError, ValueError):
                pass
        # Restore the last driver so the HMI/cloud show it after a restart,
        # until the operator taps a card again.
        for rk in ("driver_name", "driver_tag", "rfid_uid"):
            if _persist.get(rk):
                runtime[rk] = _persist[rk]
        if "driver_present" in _persist:
            runtime["driver_present"] = bool(_persist["driver_present"])
    print(f"[state] loaded {path}", flush=True)
    return _persist


def save_state():
    """Persist current geometry/target plus whatever is in _persist (atomically)."""
    if _state_path is None:          # load_state() not called (tests) -- no-op
        return
    with _lock:
        _persist["l1"] = cfg["L1"]
        _persist["l2"] = cfg["L2"]
        _persist["target_depth"] = cfg["target_depth"]
        _persist["depth_offset"] = cfg["depth_offset"]
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
        depth = compute_depth(boom["angle_deg"], stick["angle_deg"]) - cfg["depth_offset"]
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
            "hse_picture": rt["hse_picture"],
            "camera_status": rt["camera_status"],
            "driver_name": rt["driver_name"],
            "rfid_uid": rt["rfid_uid"],
            "driver_present": rt["driver_present"],
            "cloud_ok": rt["cloud_ok"], "hmi_ok": rt["hmi_ok"],
            "boom_age_ms": boom_age, "stick_age_ms": stick_age,
            "location": loc,
        }


def set_driver(tag, name):
    """Record the active driver (from an RFID scan or POST /rfid).

    A known tag (found in Driver_name_database.csv) shows the real name and sets
    driver_present=True; an unknown tag shows "Unknown Driver" with
    driver_present=False. The active driver is persisted so it survives a restart
    until the next card tap.
    """
    known = bool(name)
    resolved = name if known else "Unknown Driver"
    with _lock:
        runtime["driver_tag"] = str(tag)
        runtime["rfid_uid"] = str(tag)
        runtime["driver_name"] = resolved
        runtime["driver_present"] = known
        _persist["driver_name"] = resolved
        _persist["driver_tag"] = str(tag)
        _persist["rfid_uid"] = str(tag)
        _persist["driver_present"] = known
    save_state()
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
    """Set the HSE (person) alarm over HTTP (manual fallback).

    The primary path is the TCP socket from ezviz_camera/rpi_person_zone_alarm.py
    (see HseSocketServer); this endpoint remains for testing / other detectors:
    POST {"active": true|false, "picture": "/abs/path.jpg" (optional)}. A given
    picture is uploaded + attached to the cloud feature like a socket alarm.
    """
    d = request.get_json(silent=True) or {}
    active = bool(d.get("active", d.get("alarm", False)))
    picture = str(d.get("picture") or "")
    with _lock:
        runtime["hse_alarm"] = active
        if picture:
            runtime["hse_picture"] = os.path.basename(picture)
    if active and picture:
        _attach_q.put((picture, 0))
    print(f"[hse] alarm set to {active} via POST /hse"
          f"{'  picture=' + picture if picture else ''}", flush=True)
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
        # Saved geometry baseline for a BACK-revert (refreshed on entry + SAVE).
        with _lock:
            self.saved_L1, self.saved_L2 = cfg["L1"], cfg["L2"]
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
                self._enter_settings()
            elif val != TRIGGER and self.in_settings:
                self.in_settings = False
            return
        if addr == VP_DEPTH_OFFSET and val == OFFSET_TRIGGER:   # main-page tare key
            self.dwin.write_single_reg(addr, 0x0000, ack=False)  # consume press
            self._tare_depth()
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
            # Beam/stick: apply to the LIVE geometry so depth reflects it at once,
            # but do NOT persist -- SAVE writes to disk, BACK reverts (working vs
            # saved copy). ssid/pw are just cached until SAVE.
            if addr in (VP_BEAM_LEN, VP_STICK_LEN):
                which = "BEAM" if addr == VP_BEAM_LEN else "STICK"
                print(f"[hmi] {which} LENGTH read from HMI: '{text}' mm", flush=True)
                if text:
                    self._apply_length_live(addr, text)
            # Confirm EVERY settings input with a 1-second beep (spec).
            self._confirm_beep()
            return
        # Any other 0x83 frame reaching here is a VP this program does not know.
        # Log it loudly: if the DGUS project uploads beam/stick/save on a
        # different VP than expected, THIS line is how you find out.
        data = event_bytes(ev)
        print(f"[hmi] recv UNHANDLED VP 0x{addr:04X} value=0x{val:04X} "
              f"data={data.hex()} text='{decode_text(data)}'", flush=True)

    def _enter_settings(self):
        """Panel switched to the settings page. Snapshot the saved geometry for a
        possible BACK-revert, reset the working cache to what is on disk, and
        prefill the Wi-Fi SSID field with the current (saved) network name."""
        self.in_settings = True
        with _lock:
            self.saved_L1, self.saved_L2 = cfg["L1"], cfg["L2"]
            self.cache = {
                VP_BEAM_LEN:  str(_persist.get("beam_len_mm", "")),
                VP_STICK_LEN: str(_persist.get("stick_len_mm", "")),
                VP_SSID:      str(_persist.get("wifi_ssid", "")),
                VP_PASSWORD:  str(_persist.get("wifi_password", "")),
            }
            ssid = self.cache[VP_SSID]
        try:                                         # show the current SSID
            put_text(self.dwin, VP_SSID, ssid, SSID_LEN, ack=False)
        except Exception as e:
            print(f"[hmi] SSID prefill failed: {e}", flush=True)
        print("[hmi] panel entered SETTINGS", flush=True)

    def _tare_depth(self):
        """Main-page tare key (VP 0x1000 == 0x00FF): set the zero offset to the
        current depth so the panel reads 0 at this position (persisted)."""
        with _lock:
            raw = compute_depth(state["beam"]["angle_deg"],
                                state["stick"]["angle_deg"])
            cfg["depth_offset"] = raw
        save_state()
        try:
            self.dwin.buzzer(BuzzerDuration.BUZZ_1SEC, ack=False)
        except Exception:
            pass
        print(f"[hmi] depth TARED: offset={raw:.3f} m (VP 0x1000)", flush=True)

    def _confirm_beep(self):
        """1-second buzzer confirming the panel pushed a settings value (spec)."""
        try:
            self.dwin.buzzer(BuzzerDuration.BUZZ_1SEC, ack=False)
        except Exception:
            pass

    def _apply_length_live(self, addr, text):
        """Apply a beam/stick length to the LIVE geometry (mm -> m) so depth
        updates immediately. Persisting waits for SAVE; BACK reverts. A
        non-numeric value is logged and ignored."""
        which = "beam (L1)" if addr == VP_BEAM_LEN else "stick (L2)"
        try:
            mm = float(text)
        except ValueError:
            print(f"[hmi] {which} length '{text}' not numeric -- ignored", flush=True)
            return
        metres = mm / 1000.0
        with _lock:
            if addr == VP_BEAM_LEN:
                cfg["L1"] = metres
            else:
                cfg["L2"] = metres
        print(f"[hmi] {which} length applied LIVE: {mm:.0f} mm ({metres:.3f} m) "
              f"-- not saved until SAVE", flush=True)

    def _save(self):
        """SAVE button: persist the working copy (geometry + Wi-Fi) to disk and
        make it the new saved baseline, then return to the main page."""
        beam = self.cache.get(VP_BEAM_LEN, "")
        stick = self.cache.get(VP_STICK_LEN, "")
        ssid = self.cache.get(VP_SSID, "")
        pw = self.cache.get(VP_PASSWORD, "")
        print(f"[hmi] SAVE beam={beam!r}mm stick={stick!r}mm "
              f"ssid={ssid!r} pw={'*' * len(pw)}", flush=True)
        with _lock:
            try:                                     # keep geometry in sync w/ text
                if beam:
                    cfg["L1"] = float(beam) / 1000.0
                if stick:
                    cfg["L2"] = float(stick) / 1000.0
            except ValueError:
                print("[hmi] beam/stick not numeric -- geometry unchanged", flush=True)
            _persist.update(beam_len_mm=beam, stick_len_mm=stick,
                            wifi_ssid=ssid, wifi_password=pw)
            self.saved_L1, self.saved_L2 = cfg["L1"], cfg["L2"]   # new baseline
        save_state()
        # Wi-Fi SSID/pw are persisted to the state file ONLY (operator choice). To
        # actually switch the Pi's network, apply ssid/pw to NetworkManager /
        # wpa_supplicant here.
        self.dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
        self.dwin.goto_page(self.args.main_page, ack=False)
        self.in_settings = False

    def _cancel(self):
        """BACK button: discard edits. Restore live geometry to the saved
        baseline, reload the working cache from disk, and write the saved
        beam/stick/ssid back to the panel so the fields show the unchanged
        values (spec)."""
        with _lock:                                  # revert live geometry + cache
            cfg["L1"], cfg["L2"] = self.saved_L1, self.saved_L2
            self.cache = {
                VP_BEAM_LEN:  str(_persist.get("beam_len_mm", "")),
                VP_STICK_LEN: str(_persist.get("stick_len_mm", "")),
                VP_SSID:      str(_persist.get("wifi_ssid", "")),
                VP_PASSWORD:  str(_persist.get("wifi_password", "")),
            }
            beam, stick, ssid = (self.cache[VP_BEAM_LEN],
                                 self.cache[VP_STICK_LEN], self.cache[VP_SSID])
        try:                                         # revert the on-screen fields
            put_text(self.dwin, VP_BEAM_LEN,  beam,  LEN_TEXT_LEN, ack=False)
            put_text(self.dwin, VP_STICK_LEN, stick, LEN_TEXT_LEN, ack=False)
            put_text(self.dwin, VP_SSID,      ssid,  SSID_LEN, ack=False)
        except Exception as e:
            print(f"[hmi] revert write-back failed: {e}", flush=True)
        self.dwin.buzzer(BuzzerDuration.BUZZ_250MSEC, ack=False)
        self.dwin.goto_page(self.args.main_page, ack=False)
        self.in_settings = False
        print("[hmi] BACK: edits discarded, fields restored", flush=True)

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
        ("hse_alarm", "Integer"), ("last_alarm_picture", "String"),
        ("updated_at", "String"),
    ]
    TABLE_FIELDS = [
        ("ts", "String"), ("driver", "String"), ("depth", "Float"),
        ("target_depth", "Float"), ("over_dig_alarm", "Integer"),
        ("sensor_alarm", "Integer"), ("hse_alarm", "Integer"),
        ("lat", "Float"), ("lon", "Float"), ("event", "String"),
    ]
    # Driver Operation History (requirement #6): one row per driver session,
    # its alarm counters updated live on alarm events during the session.
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
        # Prefer the session-less API KEY (contract §1): a password login evicts
        # whoever is using the GeoMind portal AND our long-running session gets
        # revoked the moment anyone else logs in ("Session has been revoked").
        # The key rides as a ?apikey= query param, so keep TLS verification ON
        # (pass --insecure only for a self-signed tenant cert). Never log the key.
        verify = not self.args.insecure
        if self.args.geomind_apikey:
            self.client = GeoboxClient(host=self.args.geomind_host,
                                       apikey=self.args.geomind_apikey,
                                       verify=verify)
            print(f"[cloud] authenticated to {self.args.geomind_host} "
                  f"via API key (session-less)", flush=True)
        elif self.args.geomind_user and self.args.geomind_pass:
            print("[cloud] WARNING: no --geomind-apikey/$GEOMIND_APIKEY set; "
                  "falling back to PASSWORD login. This evicts the portal user "
                  "and is revoked when anyone else logs in. Use the API key.",
                  flush=True)
            self.client = GeoboxClient(host=self.args.geomind_host,
                                       username=self.args.geomind_user,
                                       password=self.args.geomind_pass,
                                       verify=verify)
            print(f"[cloud] authenticated to {self.args.geomind_host} "
                  f"as {self.args.geomind_user} (password)", flush=True)
        else:
            raise RuntimeError(
                "no GeoMind credentials: set --geomind-apikey (or $GEOMIND_APIKEY), "
                "or provide --geomind-user/--geomind-pass")

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
                               "hse_alarm": 0, "last_alarm_picture": "",
                               "updated_at": _now_iso()},
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

        # ---- resume an open session left by a previous run (persistence) ----
        self._resume_session(cloud.get("open_session"))

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
        # 1) read the target depth the operator set in the GEOMind web UI.
        #    Read at the STORED CRS (no out_srid=4326): out_srid makes the SDK
        #    call feature.transform(), which needs the geobox[geometry] extra
        #    (shapely+pyproj) and raises on a still-empty geometry -- it would
        #    crash the poll loop on a stock Pi. We overwrite the geometry on
        #    every push anyway, so the read projection is irrelevant.
        feature = self.layer.get_feature(self.feature_id)
        props = dict(feature.data.get("properties") or {})
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
                     hse_alarm=int(rt["hse_alarm"]),
                     last_alarm_picture=rt["hse_picture"],
                     updated_at=_now_iso())
        # Write the FULL GeoJSON payload with srid=4326: feature.update() forces
        # ?in_srid=4326 so the server reprojects our WGS84 [lon,lat] into its
        # stored EPSG:3857. An untagged write would treat degrees as metres and
        # grossly mislocate the marker. The payload MUST carry a "geometry" key
        # or the SDK raises KeyError 'geometry'.
        feature.update({"type": "Feature",
                        "geometry": {"type": "Point", "coordinates": [lon, lat]},
                        "properties": props}, srid=4326)

        # 3) upload + attach any queued HSE alarm pictures to the feature
        self._attach_pictures(feature, lon, lat)

        # 4) driver-session history: open/close sessions + count alarm events
        self._update_session(rt)

        # 5) log a table row ONLY on a driver change or an alarm edge
        self._maybe_log(st, rt, lat, lon, target)

    def _attach_pictures(self, feature, lon=0, lat=0):
        """Upload queued HSE-alarm evidence JPGs (sent over the camera TCP link)
        and attach each to the status feature, so the alarm picture is visible
        on the cloud object. A failed picture is retried on later cycles, up to
        3 attempts, then dropped with a log."""
        while True:
            try:
                path, tries = _attach_q.get_nowait()
            except queue.Empty:
                return
            name = os.path.basename(path)
            # The web-app gallery only lists attachments whose filename starts
            # with 'alarm_'. The camera script already names them that way; warn
            # (don't rename the on-disk file) if a detector ever sends otherwise.
            if not name.startswith("alarm_"):
                print(f"[cloud] WARNING: HSE picture '{name}' does not start with "
                      f"'alarm_'; it may not appear in the web-app gallery", flush=True)
            try:
                if not os.path.exists(path):
                    print(f"[cloud] alarm picture not found: {path} -- skipped "
                          f"(the camera script must run on THIS machine)", flush=True)
                    continue
                fobj = self.client.upload_file(path=path, scan_archive=False)
                self.layer.create_attachment(
                    name=os.path.splitext(name)[0], loc_x=lon, loc_y=lat,
                    file=fobj, feature=feature, display_name=name,
                    description=f"HSE person-in-zone alarm evidence ({_now_iso()})")
                print(f"[cloud] alarm picture attached to feature: {name}", flush=True)
            except Exception as e:
                if tries + 1 < 3:
                    _attach_q.put((path, tries + 1))
                    print(f"[cloud] picture attach failed ({e}); will retry", flush=True)
                else:
                    print(f"[cloud] picture attach failed 3x ({e}); dropped: {name}",
                          flush=True)

    def _maybe_log(self, st, rt, lat, lon, target):
        # Log a row ONLY on a change: a new driver, an alarm turning ON (a new
        # event) or an alarm turning OFF (cleared/solved). A continuous, unchanged
        # alarm adds NO rows (requirement: don't update while it stays active).
        key = (bool(rt["over_dig_alarm"]), bool(rt["sensor_alarm"]),
               bool(rt["hse_alarm"]), rt["driver_tag"])
        first = self.last_logged is None
        changed = (not first) and key != self.last_logged
        if not (first or changed):
            return
        # One row PER event token (contract §2.2 single-token enum -- never a
        # comma-joined string the bridge can't parse). Each token flips exactly
        # ONE component of the state key, so we advance the log cursor
        # (last_logged) only past the tokens whose row actually landed. On the
        # first failed row we stop: the remaining edge stays live and is retried
        # next cycle -- no lost edge, no duplicate row (fixes the non-atomic
        # "advance if any succeeded" trap).
        steps = self._event_steps(key, first)
        advanced = None if first else list(self.last_logged)
        all_ok = True
        for token, idx, newval in steps:
            try:
                self.table.create_row(
                    ts=_now_iso(), driver=rt["driver_name"], depth=st["depth_m"],
                    target_depth=target, over_dig_alarm=int(rt["over_dig_alarm"]),
                    sensor_alarm=int(rt["sensor_alarm"]), hse_alarm=int(rt["hse_alarm"]),
                    lat=lat, lon=lon, event=token)
                print(f"[cloud] row '{token}' logged (depth={st['depth_m']:.3f}m)", flush=True)
                if idx is not None:
                    advanced[idx] = newval
            except Exception as e:
                all_ok = False
                print(f"[cloud] row log failed ({token}); will retry next cycle: {e}",
                      flush=True)
                break     # leave the rest of the edge un-advanced -> retried
        if first:
            if all_ok:
                self.last_logged = key      # init landed
        else:
            self.last_logged = tuple(advanced)   # only past tokens that landed

    def _event_steps(self, key, first):
        """Ordered [(token, component_index_or_None, new_value), ...] walking
        last_logged -> key one component at a time. Each alarm/driver token flips
        exactly one key component so a per-row write can advance the cursor
        precisely as far as it succeeded (contract §2.2 single-token enum)."""
        if first:
            return [("init", None, None)]
        prev = self.last_logged
        over, sensor, hse, tag = key
        steps = []
        if prev[0] != over:
            steps.append(("over_dig_on" if over else "over_dig_off", 0, over))
        if prev[1] != sensor:
            steps.append(("sensor_alarm_on" if sensor else "sensor_alarm_off", 1, sensor))
        if prev[2] != hse:
            steps.append(("hse_on" if hse else "hse_off", 2, hse))
        if prev[3] != tag:
            steps.append(("driver_change", 3, tag))
        return steps

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
            self._persist_open_session(rt["driver_tag"])
        except Exception as e:
            self.session_row = None
            print(f"[cloud] session open failed: {e}", flush=True)

    def _resume_session(self, saved):
        """Re-attach to an open session row persisted by a previous run so its
        counters continue and it can still be closed after a restart."""
        if not saved or saved.get("row_id") is None:
            return
        try:
            self.session_row = self.session_table.get_row(saved["row_id"])
            self.session_key = saved.get("key")
            counts = {"total_alarms": 0, "hse_alarm_count": 0, "over_dig_alarm_count": 0}
            counts.update(saved.get("counts", {}))
            self.session_counts = counts
            with _lock:
                self.session_edges = {"over_dig_alarm": bool(runtime["over_dig_alarm"]),
                                      "hse_alarm": bool(runtime["hse_alarm"])}
            print(f"[cloud] resumed open session (row {saved['row_id']}, "
                  f"driver_tag={self.session_key})", flush=True)
        except Exception as e:
            self.session_row = None
            print(f"[cloud] session resume failed ({e}); will open a fresh one",
                  flush=True)

    def _persist_open_session(self, key):
        if self.session_row is None:
            return
        with _lock:
            _persist.setdefault("cloud", {})["open_session"] = {
                "row_id": getattr(self.session_row, "id", None),
                "key": key, "counts": dict(self.session_counts)}
        save_state()

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
        with _lock:
            cloud = _persist.setdefault("cloud", {})
            if isinstance(cloud.get("open_session"), dict):
                cloud["open_session"]["counts"] = dict(self.session_counts)
        save_state()

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
        with _lock:
            _persist.setdefault("cloud", {}).pop("open_session", None)
        save_state()

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
            except AuthenticationError as e:
                # A revoked/expired session (e.g. someone logged into the portal
                # while we were on password auth) lands here. Re-authenticate and
                # re-resolve the objects. The API-key path is session-less and
                # should never hit this -- the typed reconnect is defence in depth.
                with _lock:
                    runtime["cloud_ok"] = False
                print(f"[cloud] auth lost ({e}); reconnecting...", flush=True)
                try:
                    self.connect()
                    self.ensure_objects()
                    print("[cloud] reconnected", flush=True)
                except Exception as re:
                    print(f"[cloud] reconnect failed ({re}); retry next cycle",
                          flush=True)
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
    """RFID tag -> driver name, loaded from a .xlsx or .csv on the cloud (or a
    local file). XLSX is preferred because Excel keeps a big UID as a real number
    (14517093224), whereas a CSV round-tripped through Excel corrupts it into
    scientific notation ("1.45E+10") and loses digits.

    Columns are auto-detected (space/underscore/case-insensitive): a tag column
    (uid/tag/rfid/card/id) and a name column (name/driver/operator), falling back
    to the first two columns.
    """
    # Column names are matched after lowercasing + stripping spaces/underscores,
    # so "UID", "Driver Name", "driver_name" all resolve correctly.
    TAG_COLS = ("tag", "uid", "rfid", "card", "id", "tagid", "cardid")
    NAME_COLS = ("name", "driver", "drivername", "operator", "fullname")

    def __init__(self, args):
        self.args = args
        self.map = {}
        self._lock = threading.Lock()

    @staticmethod
    def _norm(tag):
        return str(tag).strip().lower().replace(":", "").replace(" ", "")

    @staticmethod
    def _normcol(col):
        return str(col).lower().strip().replace(" ", "").replace("_", "")

    def _parse(self, text):
        reader = csv.DictReader(io.StringIO(text))
        cols = reader.fieldnames or []
        if not cols:
            return {}
        low = {self._normcol(c): c for c in cols}
        tag_col = next((low[c] for c in self.TAG_COLS if c in low), cols[0])
        name_col = next((low[c] for c in self.NAME_COLS if c in low),
                        cols[1] if len(cols) > 1 else cols[0])
        out = {}
        for row in reader:
            tag = (row.get(tag_col) or "").strip()
            name = (row.get(name_col) or "").strip()
            if not tag:
                continue
            if "e+" in tag.lower():
                # e.g. "5.83718E+11": Excel saved a big UID in scientific notation
                # and lost digits, so it will NEVER match a real scan. Warn loudly.
                print(f"[rfid] WARNING: UID {tag!r} looks Excel-mangled (scientific "
                      f"notation); re-enter it as TEXT with the full digits",
                      flush=True)
            out[self._norm(tag)] = name
        return out

    def refresh(self, client=None):
        """(Re)load the directory from the cloud file, falling back to the local
        file. Handles both .xlsx and .csv (read as raw bytes so binary .xlsx is
        preserved)."""
        data, name = None, None
        if client and self.args.driver_csv:
            try:
                files = client.get_files_by_name(self.args.driver_csv)
            except Exception as e:
                files = None
                print(f"[rfid] cloud file lookup failed: {e}", flush=True)
            if files:
                resp = client.get(f"{files[0].endpoint}download/", stream=True)
                data, name = resp.content, self.args.driver_csv     # raw bytes
            else:
                print(f"[rfid] cloud file '{self.args.driver_csv}' not found", flush=True)
        if data is None and self.args.driver_csv_local:
            try:
                with open(self.args.driver_csv_local, "rb") as fh:
                    data = fh.read()
                name = self.args.driver_csv_local
            except OSError as e:
                print(f"[rfid] local file read failed: {e}", flush=True)
        if data is None:
            return
        try:
            new = self._parse_any(data, name)
        except Exception as e:
            print(f"[rfid] driver file parse failed: {e}", flush=True)
            return
        with self._lock:
            self.map = new
        print(f"[rfid] driver directory loaded: {len(new)} tags", flush=True)

    def _parse_any(self, data, name):
        """Dispatch to the XLSX or CSV parser based on the file name."""
        if name and str(name).lower().endswith((".xlsx", ".xlsm")):
            return self._parse_xlsx(data)
        if isinstance(data, bytes):
            data = data.decode("utf-8-sig", "replace")
        return self._parse(data)

    @staticmethod
    def _cell_to_tag(val):
        """Stringify a spreadsheet cell as an exact tag (no float formatting)."""
        if val is None or isinstance(val, bool):
            return ""
        if isinstance(val, float) and val.is_integer():
            return str(int(val))
        if isinstance(val, int):
            return str(val)
        return str(val).strip()

    def _parse_xlsx(self, data):
        """Parse an .xlsx driver book with openpyxl. Excel stores big UIDs as real
        integers, so there is no scientific-notation truncation like a CSV."""
        try:
            import openpyxl
        except Exception as e:
            raise RuntimeError(f"reading .xlsx needs openpyxl ({e}); "
                               f"run: pip install openpyxl")
        wb = openpyxl.load_workbook(io.BytesIO(data), read_only=True, data_only=True)
        ws = wb.active
        rows = ws.iter_rows(values_only=True)
        header = None
        for r in rows:
            if r and any(c is not None for c in r):
                header = [(str(c).strip() if c is not None else "") for c in r]
                break
        if not header:
            return {}
        low = {self._normcol(c): i for i, c in enumerate(header)}
        tag_idx = next((low[c] for c in self.TAG_COLS if c in low), 0)
        name_idx = next((low[c] for c in self.NAME_COLS if c in low),
                        1 if len(header) > 1 else 0)
        out = {}
        for r in rows:
            if not r:
                continue
            tag = self._cell_to_tag(r[tag_idx]) if tag_idx < len(r) else ""
            nm = (str(r[name_idx]).strip()
                  if name_idx < len(r) and r[name_idx] is not None else "")
            if tag:
                out[self._norm(tag)] = nm
        return out

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


# ==================================================== camera HSE link (TCP) ===
# The person detector is NOT in this process any more: the field-proven
# standalone script ezviz_camera/rpi_person_zone_alarm.py owns the camera and
# the YOLO model. It connects to THIS program over a TCP socket (--hse-port)
# and pushes newline-delimited JSON messages:
#     {"type": "hello", "who": "rpi_person_zone_alarm"}
#     {"type": "ping"}                                        (every ~10 s)
#     {"type": "hse", "active": true, "picture": "/abs/path/alarm_x.jpg"}
#     {"type": "hse", "active": false}
# On "hse" the server sets runtime["hse_alarm"] (the HMI thread mirrors it to
# panel VP 0x0400 + buzzer, the cloud thread pushes it) and, when a picture
# path is included, queues the JPG so the cloud thread uploads it and attaches
# it to the status feature. POST /hse stays as a manual fallback.

# Alarm-evidence JPGs waiting to be uploaded + attached by the cloud thread.
# Items are (path, tries) tuples.
_attach_q = queue.Queue()


def _set_camera_status(text):
    """Publish the camera-link state (shown in GET /status as camera_status)
    and log it -- so 'person detection not working' is diagnosable remotely."""
    with _lock:
        runtime["camera_status"] = text
    print(f"[hse-link] {text}", flush=True)


class HseSocketServer:
    """TCP server for the camera script's alarm link (one client at a time).

    Fail-safe like the old in-process detector: a client disconnect or
    --hse-link-timeout seconds of silence (the client pings every ~10 s)
    clears the HSE alarm, so a dead camera process can never leave the panel
    alarm + buzzer stuck on.
    """

    def __init__(self, args):
        self.args = args

    def _handle_line(self, line, addr):
        try:
            msg = json.loads(line)
        except ValueError:
            print(f"[hse-link] bad JSON from {addr}: {line[:120]!r}", flush=True)
            return
        mtype = msg.get("type")
        if mtype == "hello":
            print(f"[hse-link] client hello: {msg.get('who', '?')} from {addr}",
                  flush=True)
        elif mtype == "hse":
            active = bool(msg.get("active"))
            picture = str(msg.get("picture") or "")
            with _lock:
                runtime["hse_alarm"] = active
                if picture:
                    runtime["hse_picture"] = os.path.basename(picture)
            if active and picture:
                _attach_q.put((picture, 0))
            print(f"[hse-link] HSE alarm {'RAISED' if active else 'cleared'}"
                  f"{'  picture=' + picture if picture else ''}", flush=True)
            _set_camera_status("ALARM: person in zone" if active
                               else "armed (camera link up)")
        elif mtype != "ping":                    # pings just reset the timeout
            print(f"[hse-link] unknown message type {mtype!r}", flush=True)

    def _serve_client(self, conn, addr):
        conn.settimeout(self.args.hse_link_timeout)
        _set_camera_status(f"armed (camera client {addr} connected)")
        buf = b""
        while not _stop.is_set():
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                print(f"[hse-link] no message for "
                      f"{self.args.hse_link_timeout:.0f}s -- dropping client",
                      flush=True)
                return
            except OSError:
                return
            if not chunk:                        # client closed the connection
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    self._handle_line(line.decode("utf-8", "replace"), addr)

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self.args.hse_host, self.args.hse_port))
        except OSError as e:
            _set_camera_status(f"DISABLED: cannot bind "
                               f"{self.args.hse_host}:{self.args.hse_port} ({e})")
            return
        srv.listen(1)
        srv.settimeout(1.0)                      # so _stop is honoured promptly
        _set_camera_status(f"waiting for camera client on "
                           f"{self.args.hse_host}:{self.args.hse_port} "
                           f"(run ezviz_camera/rpi_person_zone_alarm.py)")
        while not _stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError as e:
                print(f"[hse-link] accept error: {e}", flush=True)
                continue
            with conn:
                self._serve_client(conn, f"{addr[0]}:{addr[1]}")
            # Client gone: fail safe -- never leave the alarm stuck on.
            with _lock:
                was = runtime["hse_alarm"]
                runtime["hse_alarm"] = False
            _set_camera_status("camera client disconnected"
                               + (" -- HSE alarm cleared" if was else "")
                               + "; waiting for reconnect")
        srv.close()


# ===================================================================== main ===
def main():
    ap = argparse.ArgumentParser(
        description="Integrated excavation monitor: ESP32 ingest + DWIN HMI + "
                    "GEOMind cloud + RFID driver id")
    # HTTP (ESP32 ingest server). Bind all interfaces by default: a hardcoded IP
    # crashes with EADDRNOTAVAIL when the Pi's DHCP address changes; 0.0.0.0
    # always binds and the ESP32 units still POST to the Pi's real IP.
    ap.add_argument("--http-host", default="0.0.0.0")
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
    # cloud (GEOMind). PREFER the session-less API key (contract §1): keep it in
    # the environment / a secret store, never in git or logs. Password login is a
    # last resort -- it evicts the portal user and is revoked on any other login.
    ap.add_argument("--geomind-host", default="https://app.geo-mind.ai")
    ap.add_argument("--geomind-apikey",
                    default=os.environ.get("GEOMIND_APIKEY")
                    or os.environ.get("PDO_DEVICE_APIKEY"),
                    help="GeoMind API key (default: $GEOMIND_APIKEY / "
                         "$PDO_DEVICE_APIKEY); session-less, preferred over password")
    ap.add_argument("--geomind-user", default="pdo.excavator")
    ap.add_argument("--geomind-pass", default=os.environ.get("GEOMIND_PASS"),
                    help="password fallback (default: $GEOMIND_PASS); used only "
                         "when no API key is given")
    ap.add_argument("--device-id", default="excavator1",
                    help="names the per-device cloud objects: <id>_status, <id>_log")
    ap.add_argument("--cloud-interval", type=float, default=5.0,
                    help="seconds between cloud read-target/push-feature cycles")
    ap.add_argument("--row-interval", type=float, default=10.0,
                    help="seconds between heartbeat table rows")
    ap.add_argument("--insecure", action="store_true", help="skip TLS verification")
    ap.add_argument("--no-cloud", action="store_true", help="disable cloud sync")
    # RFID / driver CSV
    ap.add_argument("--driver-csv", default="Driver_name_database.xlsx",
                    help="name of the tag->driver file on the cloud (.xlsx or .csv)")
    ap.add_argument("--driver-csv-local",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "Driver_name_database.xlsx"),
                    help="local tag->driver file (.xlsx or .csv); loaded at startup "
                         "and used as a fallback when the cloud file is unreachable")
    ap.add_argument("--csv-refresh", type=float, default=300.0,
                    help="seconds between driver-directory refreshes")
    ap.add_argument("--no-rfid", action="store_true", help="disable the RFID thread")
    # HMI alarm VPs (panel-side data variables; add matching VPs in the DGUS project)
    ap.add_argument("--overdig-vp", type=lambda x: int(x, 0), default=VP_OVERDIG_ALARM,
                    help="VP for the over-dig alarm flag (default 0x0401)")
    ap.add_argument("--hse-vp", type=lambda x: int(x, 0), default=VP_HSE_ALARM,
                    help="VP for the HSE/person alarm flag (default 0x0400)")
    # equipment home location -- the map marker sits here until a live GNSS fix
    ap.add_argument("--home-lat", type=float, default=23.5900,
                    help="fallback latitude for the map marker (default: Muscat, Oman)")
    ap.add_argument("--home-lon", type=float, default=58.4059,
                    help="fallback longitude for the map marker (default: Muscat, Oman)")
    # camera HSE alarm link (TCP socket from ezviz_camera/rpi_person_zone_alarm.py)
    ap.add_argument("--hse-host", default="0.0.0.0",
                    help="interface the HSE alarm socket server listens on")
    ap.add_argument("--hse-port", type=int, default=5050,
                    help="TCP port for the camera script's alarm link "
                         "(rpi_person_zone_alarm.py --alarm-port must match)")
    ap.add_argument("--hse-link-timeout", type=float, default=30.0,
                    help="drop the camera client after this many s of silence "
                         "(the client pings every ~10 s)")
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
    # HSE alarm link: the standalone camera script connects here (TCP).
    threads.append(threading.Thread(target=HseSocketServer(args).run,
                                    name="hse-link", daemon=True))

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
