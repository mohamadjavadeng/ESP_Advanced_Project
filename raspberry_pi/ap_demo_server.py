#!/usr/bin/env python3
"""
Minimal demo HTTP server for the concurrent AP+STA Raspberry Pi (ap_sta_setup.py).

The ESP32 (see ../esp32_ap_client) joins the Pi's OWN access point
(SSID RPi_AP, Pi fixed IP 192.168.50.1) and exercises both verbs here:

    POST /ingest   {"device":"esp32-01","seq":N,"uptime_ms":..,"temp_c":..}
                   -> stores the payload, echoes it back with a server timestamp
    GET  /latest   -> returns the last payload that was POSTed
    GET  /         -> health / info

Same Flask style as sensor_receiver.py (threaded, quiet werkzeug access log).
Binds 0.0.0.0 so it answers on BOTH the AP (uap0 / 192.168.50.1) and the home
WiFi (wlan0).

    Run:  pip install flask  &&  python3 ap_demo_server.py
"""
import argparse
import logging
import threading
from datetime import datetime, timezone

from flask import Flask, jsonify, request

app = Flask(__name__)
# Quiet Werkzeug's per-request line; we print our own parsed line in ingest().
logging.getLogger("werkzeug").setLevel(logging.ERROR)

_lock = threading.Lock()
_latest = {}     # last payload we received
_count = 0       # how many POSTs seen so far


@app.post("/ingest")
def ingest():
    """Receive one JSON sample from the ESP32."""
    global _latest, _count
    data = request.get_json(silent=True) or {}
    with _lock:
        _count += 1
        _latest = dict(data)
        _latest["_received_at"] = datetime.now(timezone.utc).isoformat()
        _latest["_seq_server"] = _count
        snap = dict(_latest)
    print(f"POST /ingest #{snap['_seq_server']:<5} {data}", flush=True)
    return jsonify(ok=True, stored=snap)


@app.get("/latest")
def latest():
    """Return whatever the ESP32 last POSTed (proves the GET round-trip)."""
    with _lock:
        return jsonify(_latest or {"message": "no data yet"})


@app.get("/")
def root():
    with _lock:
        n = _count
    return jsonify(status="up", posts=n,
                   endpoints=["POST /ingest", "GET /latest", "GET /"])


def main():
    ap = argparse.ArgumentParser(description="ESP32 <-> Pi AP demo server")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    print(f"AP demo server on {args.host}:{args.port} "
          f"(POST /ingest, GET /latest, GET /)")
    # threaded=True so the ESP32 and any browser poller don't block each other.
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
