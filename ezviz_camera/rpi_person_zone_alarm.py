#!/usr/bin/env python3
"""
rpi_person_zone_alarm.py — EZVIZ H8c person-in-zone intrusion alarm (Raspberry Pi 4B).

STEP 2: YOLO person detection restricted to a user-defined zone, with a dwell
timer. If a person stays inside the zone continuously for >= --dwell seconds
(default 3), an ALARM is logged to the command line and an annotated snapshot is
saved as evidence. Runs headless (over SSH, no display).

Detection uses YOLOv4-tiny through OpenCV's DNN module (cv2.dnn) on CPU —
NO PyTorch / NO ultralytics / NO CUDA. Suits a Pi 4B and the lean
opencv-python-headless install. Model files (~23 MB) auto-download on first run
into ./models (or fetch manually — see below).

How false alarms are avoided (the "must be confident" part):
  * per-detection confidence threshold           (--conf,    default 0.50)
  * non-max suppression removes duplicate boxes   (--nms,     default 0.40)
  * a person counts as "in zone" only if its box overlaps the zone by
    >= --overlap of the box area                  (--overlap, default 0.30)
  * a short --grace window tolerates brief YOLO dropouts / occlusion so a
    one-frame flicker does NOT reset the dwell timer (--grace, default 1.0s)
  * the dwell timer uses WALL-CLOCK time, so frame-rate and frame-skips do not
    change the 3-second requirement

Zone is given as fractions of the frame, so it is independent of sub/main res:
    --zone x1,y1,x2,y2     each 0..1, top-left origin.  Default 0,0,1,1 (whole frame)
Calibrate by opening any saved alarms/alarm_*.jpg — the yellow box is the zone.

Model files (auto-downloaded; or pre-fetch on the Pi):
    mkdir -p models
    wget -O models/yolov4-tiny.cfg     https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg
    wget -O models/yolov4-tiny.weights https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights

INTEGRATION: this script keeps a TCP link to monitoring_control.py
(--alarm-host, default 127.0.0.1, port 5050 = its --hse-port) and sends
newline-JSON alarm edges + the saved evidence picture's path; the controller
drives the DWIN panel + buzzer and attaches the picture to the GEOMind
feature. Pass --alarm-host "" to disable the link.

Run (in the venv, headless):
    python3 rpi_person_zone_alarm.py
    python3 rpi_person_zone_alarm.py --zone 0.25,0.3,0.75,0.95   # watch a sub-area
    python3 rpi_person_zone_alarm.py --dwell 3 --conf 0.6
    python3 rpi_person_zone_alarm.py --main                      # HD stream
    python3 rpi_person_zone_alarm.py --logfile alarms.log        # also log to file
    python3 rpi_person_zone_alarm.py --duration 20               # stop after 20s (testing)
Stop with Ctrl-C.
"""

import argparse
import json
import logging
import os
import socket
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

# Force FFmpeg to use TCP (reliable over WiFi) and fail fast on a stalled socket.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;5000000")

PERSON_CLASS_ID = 0  # COCO 'person'
FONT = cv2.FONT_HERSHEY_SIMPLEX

MODEL_URLS = {
    "yolov4-tiny.cfg":
        "https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg",
    "yolov4-tiny.weights":
        "https://github.com/AlexeyAB/darknet/releases/download/"
        "darknet_yolo_v4_pre/yolov4-tiny.weights",
}


# --------------------------------------------------------------------------- #
# Model
# --------------------------------------------------------------------------- #
def ensure_model(models_dir):
    """Return (cfg_path, weights_path), auto-downloading if missing."""
    os.makedirs(models_dir, exist_ok=True)
    cfg = os.path.join(models_dir, "yolov4-tiny.cfg")
    wts = os.path.join(models_dir, "yolov4-tiny.weights")
    for path, url in ((cfg, MODEL_URLS["yolov4-tiny.cfg"]),
                      (wts, MODEL_URLS["yolov4-tiny.weights"])):
        if os.path.exists(path) and os.path.getsize(path) > 0:
            continue
        logging.info("downloading %s (one-time) ...", os.path.basename(path))
        try:
            urllib.request.urlretrieve(url, path)
        except Exception as e:  # network / SSL / 404
            logging.error("auto-download failed: %s", e)
            print("\nDownload the model manually, then re-run:\n"
                  f"  mkdir -p {models_dir}\n"
                  f"  wget -O {cfg} {MODEL_URLS['yolov4-tiny.cfg']}\n"
                  f"  wget -O {wts} {MODEL_URLS['yolov4-tiny.weights']}\n",
                  file=sys.stderr)
            sys.exit(1)
    return cfg, wts


def load_model(cfg, wts, inp):
    """Build a cv2.dnn DetectionModel (YOLOv4-tiny) on CPU."""
    net = cv2.dnn.readNetFromDarknet(cfg, wts)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    model = cv2.dnn.DetectionModel(net)
    # scale 1/255, square input, swapRB (BGR->RGB); DetectionModel handles NMS.
    model.setInputParams(scale=1 / 255.0, size=(inp, inp), swapRB=True)
    return model


def detect_persons(model, frame, conf, nms):
    """Return list of (x1, y1, x2, y2, confidence) for detected people."""
    class_ids, confidences, boxes = model.detect(
        frame, confThreshold=conf, nmsThreshold=nms)
    out = []
    if len(boxes) == 0:
        return out
    ids = np.array(class_ids).reshape(-1)
    cfs = np.array(confidences).reshape(-1)
    for cid, cf, box in zip(ids, cfs, boxes):
        if int(cid) != PERSON_CLASS_ID:
            continue
        x, y, w, h = box
        out.append((int(x), int(y), int(x + w), int(y + h), float(cf)))
    return out


# --------------------------------------------------------------------------- #
# Geometry / zone
# --------------------------------------------------------------------------- #
def overlap_ratio(box, zone):
    """Intersection area / box area — how much of the person sits in the zone."""
    ix1 = max(box[0], zone[0])
    iy1 = max(box[1], zone[1])
    ix2 = min(box[2], zone[2])
    iy2 = min(box[3], zone[3])
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    barea = (box[2] - box[0]) * (box[3] - box[1])
    return inter / barea if barea > 0 else 0.0


def parse_zone(s):
    parts = s.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("zone must be x1,y1,x2,y2 (fractions 0..1)")
    v = [max(0.0, min(1.0, float(p))) for p in parts]
    x1, x2 = sorted((v[0], v[2]))
    y1, y2 = sorted((v[1], v[3]))
    return (x1, y1, x2, y2)


def zone_to_px(zone_frac, w, h):
    return (int(zone_frac[0] * w), int(zone_frac[1] * h),
            int(zone_frac[2] * w), int(zone_frac[3] * h))


# --------------------------------------------------------------------------- #
# Threaded latest-frame grabber (keeps latency low; RTSP buffers otherwise)
# --------------------------------------------------------------------------- #
def open_capture(url, attempts, wait):
    """Open RTSP, retrying — the camera's RTSP daemon boots slowly after reboot."""
    for i in range(1, attempts + 1):
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # honored by some backends
            except cv2.error:
                pass
            return cap
        cap.release()
        if i < attempts:
            logging.warning("open failed (%d/%d); retrying in %.0fs — camera "
                            "may still be booting", i, attempts, wait)
            time.sleep(wait)
    return None


class FrameGrabber(threading.Thread):
    """Continuously reads frames, keeps only the newest, reconnects on stall."""

    def __init__(self, url, attempts, wait):
        super().__init__(daemon=True)
        self.url, self.attempts, self.wait = url, attempts, wait
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.running = True
        self.cap = None

    def run(self):
        self.cap = open_capture(self.url, self.attempts, self.wait)
        if self.cap is None:
            self.running = False
            return
        fails = 0
        while self.running:
            ok, f = self.cap.read()
            if not ok or f is None:
                fails += 1
                if fails >= 30:  # ~1s of dead reads -> reconnect
                    logging.warning("stream stalled — reconnecting")
                    try:
                        self.cap.release()
                    except cv2.error:
                        pass
                    self.cap = open_capture(self.url, self.attempts, self.wait)
                    if self.cap is None:
                        logging.error("reconnect failed; giving up")
                        self.running = False
                        break
                    fails = 0
                time.sleep(0.03)
                continue
            fails = 0
            with self.lock:
                self.frame = f
                self.seq += 1

    def read(self):
        with self.lock:
            return self.seq, self.frame

    def stop(self):
        self.running = False


# --------------------------------------------------------------------------- #
# Drawing / evidence
# --------------------------------------------------------------------------- #
def annotate(frame, dets, zone_px, overlap, dwell, alarmed):
    """Return a copy of frame with zone, boxes, dwell timer and ALARM drawn."""
    f = frame.copy()
    h = f.shape[0]
    zx1, zy1, zx2, zy2 = zone_px
    cv2.rectangle(f, (zx1, zy1), (zx2, zy2), (0, 255, 255), 2)
    cv2.putText(f, "ZONE", (zx1 + 4, zy1 + 18), FONT, 0.6, (0, 255, 255), 2)
    for (x1, y1, x2, y2, cf) in dets:
        in_zone = overlap_ratio((x1, y1, x2, y2), zone_px) >= overlap
        col = (0, 0, 255) if in_zone else (0, 255, 0)  # red in-zone, green outside
        cv2.rectangle(f, (x1, y1), (x2, y2), col, 2)
        cv2.putText(f, f"person {cf:.2f}", (x1, max(12, y1 - 6)),
                    FONT, 0.5, col, 2)
    cv2.putText(f, time.strftime("%Y-%m-%d %H:%M:%S"), (8, h - 10),
                FONT, 0.6, (255, 255, 255), 2)
    status = f"in-zone {dwell:.1f}s" if dwell is not None else "clear"
    cv2.putText(f, status, (8, 24), FONT, 0.7, (255, 255, 255), 2)
    if alarmed:
        cv2.putText(f, "ALARM", (8, 60), FONT, 1.1, (0, 0, 255), 3)
    return f


def save_alarm(frame, dets, zone_px, overlap, dwell, alarms_dir):
    """Save the annotated alarm snapshot; return its ABSOLUTE path (or None).
    The path is sent over the alarm link so monitoring_control.py can upload
    the picture and attach it to the cloud feature."""
    os.makedirs(alarms_dir, exist_ok=True)
    img = annotate(frame, dets, zone_px, overlap, dwell, alarmed=True)
    fn = os.path.join(alarms_dir, time.strftime("alarm_%Y%m%d_%H%M%S.jpg"))
    if cv2.imwrite(fn, img):
        fn = os.path.abspath(fn)
        logging.info("evidence saved -> %s", fn)
        return fn
    return None


# --------------------------------------------------------------------------- #
def setup_logging(logfile):
    handlers = [logging.StreamHandler(sys.stdout)]
    if logfile:
        handlers.append(logging.FileHandler(logfile))
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=handlers)


def build_url(ip, code, sub):
    channel = "sub" if sub else "main"
    return f"rtsp://admin:{code}@{ip}:554/H264/ch1/{channel}/av_stream"


def post_hse(url, active):
    """Optional integration hook: POST {"active": bool} to monitoring_control's
    /hse endpoint so the excavation controller raises/clears the HSE alarm.
    No-op when --post-hse is not set."""
    if not url:
        return
    try:
        body = json.dumps({"active": bool(active)}).encode()
        req = urllib.request.Request(url, data=body,
                                     headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=1.0).read()
    except Exception as e:
        logging.warning("post_hse failed: %s", e)


class AlarmLink:
    """Persistent TCP link to monitoring_control.py (newline-delimited JSON).

    monitoring_control.py runs an HSE alarm socket server (--hse-port, default
    5050); this client sends the alarm edges (+ the evidence picture path) and
    a "ping" every --alarm-ping seconds so the server knows the camera is
    alive (the server clears the alarm if the link goes silent). Everything is
    best-effort with ONE silent reconnect per send: a down link must never
    stall or kill the detection loop. Disabled when host is empty ("").

    Messages sent:
        {"type": "hello", "who": "rpi_person_zone_alarm"}     on (re)connect
        {"type": "ping"}                                      keep-alive
        {"type": "hse", "active": true, "picture": "/abs/alarm_x.jpg", "ts": ..}
        {"type": "hse", "active": false}
    """

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.sock = None
        self.lock = threading.Lock()

    def _drop(self):
        try:
            if self.sock is not None:
                self.sock.close()
        except OSError:
            pass
        self.sock = None

    def _alive(self):
        """True if the socket still looks usable. Detects a server-side close
        (half-open TCP): without this, the first sendall() after the server
        dropped us "succeeds" into the void and an ALARM MESSAGE IS LOST."""
        if self.sock is None:
            return False
        try:
            self.sock.setblocking(False)
            try:
                if self.sock.recv(1, socket.MSG_PEEK) == b"":
                    return False                 # EOF: server closed the link
            except (BlockingIOError, InterruptedError):
                pass                             # nothing to read = healthy
            except OSError:
                return False
            return True
        finally:
            if self.sock is not None:
                self.sock.settimeout(3.0)

    def send(self, obj):
        """Send one JSON message; returns True on success."""
        if not self.host:
            return False
        line = (json.dumps(obj) + "\n").encode()
        with self.lock:
            err = None
            for _ in range(2):                   # 2nd try = fresh connection
                try:
                    if self.sock is not None and not self._alive():
                        self._drop()             # stale socket -> reconnect
                    if self.sock is None:
                        self.sock = socket.create_connection(
                            (self.host, self.port), timeout=3.0)
                        self.sock.settimeout(3.0)
                        hello = json.dumps({"type": "hello",
                                            "who": "rpi_person_zone_alarm"})
                        self.sock.sendall((hello + "\n").encode())
                        logging.info("alarm link connected to %s:%d",
                                     self.host, self.port)
                    self.sock.sendall(line)
                    return True
                except OSError as e:
                    err = e
                    self._drop()
            logging.warning("alarm link send failed: %s", err)
            return False

    def start_ping(self, interval):
        """Background keep-alive; also quietly re-establishes a dropped link."""
        if not self.host or interval <= 0:
            return

        def _loop():
            while True:
                time.sleep(interval)
                self.send({"type": "ping"})
        threading.Thread(target=_loop, daemon=True).start()


def main():
    ap = argparse.ArgumentParser(
        description="Person-in-zone intrusion alarm (YOLOv4-tiny / cv2.dnn, Pi 4B)")
    ap.add_argument("--ip", default="192.168.100.67", help="camera IP")
    ap.add_argument("--code", default=os.environ.get("EZVIZ_CODE", "Aman2026"),
                    help="RTSP password / verification code (or set EZVIZ_CODE)")
    ap.add_argument("--main", action="store_true",
                    help="use main (HD) stream; default sub (low-res, faster)")
    ap.add_argument("--zone", type=parse_zone, default=(0.0, 0.0, 1.0, 1.0),
                    help="watch area as x1,y1,x2,y2 fractions 0..1 (default whole frame)")
    ap.add_argument("--dwell", type=float, default=3.0,
                    help="seconds a person must stay in zone before alarm (default 3)")
    ap.add_argument("--conf", type=float, default=0.50,
                    help="detection confidence threshold 0..1 (default 0.50)")
    ap.add_argument("--nms", type=float, default=0.40,
                    help="non-max-suppression IoU threshold (default 0.40)")
    ap.add_argument("--overlap", type=float, default=0.30,
                    help="min box-in-zone overlap fraction to count as inside (default 0.30)")
    ap.add_argument("--grace", type=float, default=1.0,
                    help="seconds of absence tolerated before the timer resets (default 1.0)")
    ap.add_argument("--remind", type=float, default=15.0,
                    help="while alarming, re-log 'still present' every N s (0 = off)")
    ap.add_argument("--input", type=int, default=416,
                    help="YOLO input size (320 faster, 416 default, 608 more accurate)")
    ap.add_argument("--models-dir", default="models", help="where model files live")
    ap.add_argument("--alarms-dir", default="alarms", help="where to save evidence jpgs")
    ap.add_argument("--no-save", action="store_true", help="do not save evidence jpgs")
    ap.add_argument("--logfile", default=None, help="also append logs to this file")
    ap.add_argument("--attempts", type=int, default=5, help="RTSP open retries")
    ap.add_argument("--wait", type=float, default=4.0, help="seconds between retries")
    ap.add_argument("--duration", type=float, default=0,
                    help="stop after N seconds (0 = run until Ctrl-C; useful for testing)")
    ap.add_argument("--post-hse", default=None,
                    help='POST {"active":true/false} to this URL on alarm fire/clear, '
                         "e.g. http://127.0.0.1:5000/hse to feed monitoring_control.py")
    ap.add_argument("--alarm-host", default="127.0.0.1",
                    help="monitoring_control.py host for the TCP alarm link "
                         '("" disables); both scripts usually run on the same Pi')
    ap.add_argument("--alarm-port", type=int, default=5050,
                    help="monitoring_control.py --hse-port (default 5050)")
    ap.add_argument("--alarm-ping", type=float, default=10.0,
                    help="keep-alive ping period for the alarm link, s (0 = off)")
    args = ap.parse_args()

    setup_logging(args.logfile)

    # TCP alarm link to monitoring_control.py: alarm edges + evidence picture
    # path go over this socket; the controller drives the DWIN panel + buzzer
    # and attaches the picture to the GEOMind cloud feature.
    link = AlarmLink(args.alarm_host, args.alarm_port)
    link.start_ping(args.alarm_ping)
    if args.alarm_host:
        logging.info("alarm link target: %s:%d (monitoring_control.py --hse-port)",
                     args.alarm_host, args.alarm_port)

    if not args.code:
        logging.error("no verification code. Use --code XXXX or set EZVIZ_CODE.")
        sys.exit(1)

    cfg, wts = ensure_model(args.models_dir)
    logging.info("loading YOLOv4-tiny (cv2.dnn, CPU, input %d)...", args.input)
    model = load_model(cfg, wts, args.input)

    url = build_url(args.ip, args.code, sub=not args.main)
    logging.info("opening %s", url.replace(args.code, "******"))

    grab = FrameGrabber(url, max(1, args.attempts), args.wait)
    grab.start()

    # Wait for the first real frame.
    t_wait = time.time()
    while True:
        _, frame = grab.read()
        if frame is not None:
            break
        if not grab.is_alive() or time.time() - t_wait > 20:
            logging.error("no video after 20s — check RTSP enabled, port 554, IP, code.")
            grab.stop()
            sys.exit(1)
        time.sleep(0.1)

    h, w = frame.shape[:2]
    zone_px = zone_to_px(args.zone, w, h)
    logging.info("frame %dx%d | zone px %s | dwell %.1fs | conf %.2f | overlap %.2f",
                 w, h, zone_px, args.dwell, args.conf, args.overlap)
    logging.info("armed — watching for a person in the zone (Ctrl-C to stop)")

    # Dwell state machine (all times are wall-clock).
    t0 = None            # when the current continuous presence started
    last_seen = 0.0      # last time a person was in the zone
    alarmed = False      # has the alarm already fired for this presence
    next_remind = 0.0
    last_seq = -1
    start = time.time()

    try:
        while True:
            if args.duration and time.time() - start >= args.duration:
                logging.info("duration reached — stopping")
                break

            seq, frame = grab.read()
            if frame is None or seq == last_seq:
                if not grab.is_alive():
                    logging.error("capture thread died — stopping")
                    break
                time.sleep(0.01)
                continue
            last_seq = seq

            dets = detect_persons(model, frame, args.conf, args.nms)
            in_zone = [d for d in dets
                       if overlap_ratio(d, zone_px) >= args.overlap]
            now = time.time()

            if in_zone:
                last_seen = now
                if t0 is None:
                    t0 = now
                    best = max(d[4] for d in in_zone)
                    logging.info("person entered zone (%d, top conf %.2f)",
                                 len(in_zone), best)
                dwell = now - t0
                if dwell >= args.dwell and not alarmed:
                    alarmed = True
                    next_remind = now + args.remind
                    logging.critical(
                        "*** ALARM *** person in zone %.1fs (>= %.1fs) — %d person(s)",
                        dwell, args.dwell, len(in_zone))
                    pic = None
                    if not args.no_save:
                        pic = save_alarm(frame, dets, zone_px, args.overlap,
                                         dwell, args.alarms_dir)
                    post_hse(args.post_hse, True)
                    link.send({"type": "hse", "active": True, "picture": pic,
                               "ts": time.strftime("%Y-%m-%d %H:%M:%S")})
                elif alarmed and args.remind and now >= next_remind:
                    logging.warning("still present — %.0fs in zone", dwell)
                    next_remind = now + args.remind
            else:
                # No one in the zone this frame; tolerate brief gaps via --grace.
                if t0 is not None and (now - last_seen) > args.grace:
                    if alarmed:
                        logging.info("zone clear — alarm reset")
                        post_hse(args.post_hse, False)
                        link.send({"type": "hse", "active": False})
                    else:
                        logging.info("left zone before %.1fs dwell — no alarm",
                                     args.dwell)
                    t0 = None
                    alarmed = False
    except KeyboardInterrupt:
        logging.info("stopping (Ctrl-C)")
    finally:
        if alarmed:                       # never leave the panel alarm stuck on
            link.send({"type": "hse", "active": False})
        grab.stop()


if __name__ == "__main__":
    main()
