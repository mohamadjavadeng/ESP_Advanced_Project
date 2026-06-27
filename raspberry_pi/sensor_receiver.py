#!/usr/bin/env python3
"""
Reference Raspberry Pi receiver for the RPi-centric excavation architecture.

Both ESP32 sensor units PUSH their RAW tilt angle here over the field WiFi:

    POST /ingest
    {"id":"stick"|"beam","angle_deg":12.3,"mpu_ok":true,"seq":N,"uptime_ms":..}

This service does what the old ESP32 server used to do, but on the Pi:
  * stores the latest sample per unit (with an arrival timestamp),
  * applies sign + zero-offset calibration and computes the depth
        depth = L1*sin(boom) + L2*sin(stick),
  * raises depth_alarm when depth >= target_depth,
  * flags a sensor as STALE if no packet arrives within --stale-ms,
  * serves the live state on  GET /status  (JSON) -- SAME field names as the old
    ESP32 server, so excavation_monitor.py, the DWIN driver and the GEOMind
    uploader can all read this unchanged,
  * accepts target depth + offsets from the HMI/cloud via  POST /config.

The camera person-detector (ezviz_camera/rpi_stream.py), the DWIN HMI driver and
the GEOMind cloud uploader run as SEPARATE processes; they just GET /status here
(or `from sensor_receiver import compute_depth`). See DEPLOYMENT_GUIDE.md.

Run:
    pip install flask
    python3 sensor_receiver.py --l1 1.1 --l2 1.0 --target 1.5 --port 8080
"""
import argparse
import logging
import math
import threading
import time

from flask import Flask, jsonify, request

app = Flask(__name__)

# Quiet Werkzeug's per-request access log ("POST /ingest ... 200 -"); we print
# our own parsed, human-readable line in ingest() instead. Errors still show.
logging.getLogger("werkzeug").setLevel(logging.ERROR)

# --------------------------------------------------------------------------- #
# All shared state lives here, guarded by a lock (Flask is multi-threaded).
_lock = threading.Lock()
state = {
    "stick": {"angle_deg": 0.0, "mpu_ok": False, "seq": -1, "ts": 0.0},
    "beam":  {"angle_deg": 0.0, "mpu_ok": False, "seq": -1, "ts": 0.0},
}
cfg = {
    "L1": 1.1, "L2": 1.0,            # boom / stick lengths (m)
    "boom_sign": 1.0, "stick_sign": 1.0,
    "boom_offset_deg": 0.0, "stick_offset_deg": 0.0,
    "target_depth": 1.5,
    "stale_ms": 1500,                # a unit older than this is "stale"
}


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
    """Snapshot in the SAME schema the old ESP32 server served on /status."""
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
        return {
            "boom_deg": round(boom["angle_deg"], 2),
            "stick_deg": round(stick["angle_deg"], 2),
            "depth_m": round(depth, 3),
            "target_depth": cfg["target_depth"],
            "depth_alarm": depth_alarm,
            "sensor_ok": sensor_ok,        # False => a sensor is dead/stale (alarm!)
            "boom_age_ms": boom_age,
            "stick_age_ms": stick_age,
        }


# --------------------------------------------------------------- HTTP routes --
@app.post("/ingest")
def ingest():
    """Receive one raw sample from an ESP32 unit."""
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
    # One parsed line per packet so the terminal shows real data, not just
    # "POST /ingest 200". Depth/alarm reflect the latest pair of both units.
    st = build_status()
    print(f"[{uid:>5}] angle={angle:7.2f}  mpu_ok={mpu_ok!s:<5}  seq={seq:<6} | "
          f"depth={st['depth_m']:6.3f}m  alarm={st['depth_alarm']!s:<5}  "
          f"sensor_ok={st['sensor_ok']}", flush=True)
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
    return jsonify(ok=True, cfg=cfg)


def main():
    ap = argparse.ArgumentParser(description="ESP32 -> Pi sensor receiver")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--l1", type=float, default=cfg["L1"], help="boom length (m)")
    ap.add_argument("--l2", type=float, default=cfg["L2"], help="stick length (m)")
    ap.add_argument("--target", type=float, default=cfg["target_depth"],
                    help="target depth (m)")
    ap.add_argument("--stale-ms", type=int, default=cfg["stale_ms"],
                    help="a sensor with no packet for this long is 'stale'")
    args = ap.parse_args()
    cfg.update(L1=args.l1, L2=args.l2, target_depth=args.target,
               stale_ms=args.stale_ms)

    print(f"Receiver on :{args.port}  L1={cfg['L1']} L2={cfg['L2']} "
          f"target={cfg['target_depth']}m  (POST /ingest, GET /status)")
    # threaded=True so two ESP32s + /status pollers don't block each other.
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
