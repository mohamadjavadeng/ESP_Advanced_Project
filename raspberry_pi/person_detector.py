#!/usr/bin/env python3
"""
person_detector.py -- reusable person-in-zone detector for monitoring_control.py.

This is the person-detection alarm from ezviz_camera/rpi_person_zone_alarm.py,
repackaged as a library so the excavation controller can run it IN-PROCESS on a
background thread and get only the ALARM STATE EDGES via a callback -- it does
NOT own the serial port, the cloud client or any shared state; it just calls
`on_change(True)` when a person has dwelt in the zone long enough and
`on_change(False)` when the zone clears.

Detection = YOLOv4-tiny through OpenCV's DNN module (cv2.dnn) on CPU: no PyTorch,
no ultralytics, no CUDA -- suits a Pi 4B. Model files (~23 MB) auto-download to
`models_dir` on first use.

Confidence / anti-false-alarm tuning is identical to the standalone script:
  * per-detection confidence threshold           (conf,    default 0.50)
  * non-max suppression removes duplicate boxes   (nms,     default 0.40)
  * a person counts as "in zone" only if its box overlaps the zone by
    >= overlap of the box area                    (overlap, default 0.30)
  * a short grace window tolerates brief YOLO dropouts / occlusion so a
    one-frame flicker does NOT reset the dwell timer (grace, default 1.0s)
  * the dwell timer uses WALL-CLOCK time, independent of frame rate

cv2 + numpy are imported at module load, so import this module lazily (only when
the camera is actually enabled) -- monitoring_control.py does exactly that.
"""

import os
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
    """Return (cfg_path, weights_path), auto-downloading if missing.

    Raises RuntimeError on a failed download (the caller runs on a thread, so we
    must NOT sys.exit like the standalone script does).
    """
    os.makedirs(models_dir, exist_ok=True)
    cfg = os.path.join(models_dir, "yolov4-tiny.cfg")
    wts = os.path.join(models_dir, "yolov4-tiny.weights")
    for path, url in ((cfg, MODEL_URLS["yolov4-tiny.cfg"]),
                      (wts, MODEL_URLS["yolov4-tiny.weights"])):
        if os.path.exists(path) and os.path.getsize(path) > 0:
            continue
        try:
            urllib.request.urlretrieve(url, path)
        except Exception as e:  # network / SSL / 404
            raise RuntimeError(
                f"model download failed for {os.path.basename(path)}: {e} -- "
                f"pre-fetch it manually into {models_dir}") from e
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
    """Return (persons, n_raw). `persons` is a list of (x1,y1,x2,y2,conf) for the
    COCO 'person' class; `n_raw` is the TOTAL boxes the model returned for ALL
    classes. n_raw distinguishes 'model sees nothing' (0) from 'model sees only
    non-person objects' (>0 with persons empty) -- key for diagnosing detection."""
    class_ids, confidences, boxes = model.detect(
        frame, confThreshold=conf, nmsThreshold=nms)
    out = []
    if len(boxes) == 0:
        return out, 0
    ids = np.array(class_ids).reshape(-1)
    cfs = np.array(confidences).reshape(-1)
    for cid, cf, box in zip(ids, cfs, boxes):
        if int(cid) != PERSON_CLASS_ID:
            continue
        x, y, w, h = box
        out.append((int(x), int(y), int(x + w), int(y + h), float(cf)))
    return out, int(len(ids))


# --------------------------------------------------------------------------- #
# Geometry / zone
# --------------------------------------------------------------------------- #
def overlap_ratio(box, zone):
    """Intersection area / box area -- how much of the person sits in the zone."""
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
    """Parse 'x1,y1,x2,y2' (fractions 0..1) into a sorted, clamped tuple."""
    parts = str(s).split(",")
    if len(parts) != 4:
        raise ValueError("zone must be x1,y1,x2,y2 (fractions 0..1)")
    v = [max(0.0, min(1.0, float(p))) for p in parts]
    x1, x2 = sorted((v[0], v[2]))
    y1, y2 = sorted((v[1], v[3]))
    return (x1, y1, x2, y2)


def zone_to_px(zone_frac, w, h):
    return (int(zone_frac[0] * w), int(zone_frac[1] * h),
            int(zone_frac[2] * w), int(zone_frac[3] * h))


def build_rtsp_url(ip, code, sub=True):
    """EZVIZ / Hikvision RTSP URL. admin + verification code, sub or main stream."""
    channel = "sub" if sub else "main"
    return f"rtsp://admin:{code}@{ip}:554/H264/ch1/{channel}/av_stream"


# --------------------------------------------------------------------------- #
# Threaded latest-frame grabber (keeps latency low; RTSP buffers otherwise)
# --------------------------------------------------------------------------- #
def open_capture(url, attempts, wait, stop_event=None):
    """Open RTSP, retrying -- the camera's RTSP daemon boots slowly after reboot."""
    for i in range(1, attempts + 1):
        if stop_event is not None and stop_event.is_set():
            return None
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # honored by some backends
            except cv2.error:
                pass
            return cap
        cap.release()
        if i < attempts:
            if stop_event is not None and stop_event.wait(wait):
                return None
            elif stop_event is None:
                time.sleep(wait)
    return None


class FrameGrabber:
    """Continuously reads frames on its own thread, keeps only the newest,
    reconnects on stall. Started/stopped by PersonZoneMonitor."""

    def __init__(self, url, attempts, wait, stop_event):
        import threading
        self.url, self.attempts, self.wait = url, attempts, wait
        self.stop_event = stop_event
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.running = True
        self.cap = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        self.cap = open_capture(self.url, self.attempts, self.wait, self.stop_event)
        if self.cap is None:
            self.running = False
            return
        fails = 0
        while self.running and not self.stop_event.is_set():
            ok, f = self.cap.read()
            if not ok or f is None:
                fails += 1
                if fails >= 30:  # ~1s of dead reads -> reconnect
                    try:
                        self.cap.release()
                    except cv2.error:
                        pass
                    self.cap = open_capture(self.url, self.attempts, self.wait,
                                            self.stop_event)
                    if self.cap is None:
                        self.running = False
                        break
                    fails = 0
                time.sleep(0.03)
                continue
            fails = 0
            with self.lock:
                self.frame = f
                self.seq += 1
        try:
            if self.cap is not None:
                self.cap.release()
        except cv2.error:
            pass

    def read(self):
        with self.lock:
            return self.seq, self.frame

    def is_alive(self):
        return self.thread.is_alive()

    def stop(self):
        self.running = False


# --------------------------------------------------------------------------- #
# Monitor: runs the dwell state machine, reports alarm edges via a callback
# --------------------------------------------------------------------------- #
class PersonZoneMonitor:
    """Watches an RTSP stream for a person dwelling in a zone.

    Calls `on_change(True)` once when the dwell threshold is first reached and
    `on_change(False)` once when the zone later clears (grace-tolerant). All
    exceptions from the callback are swallowed so a slow consumer can't kill the
    detection loop.
    """

    def __init__(self, url, *, zone=(0.0, 0.0, 1.0, 1.0), dwell=3.0, conf=0.50,
                 nms=0.40, overlap=0.30, grace=1.0, input_size=416,
                 models_dir="models", on_change=None, on_log=None,
                 attempts=5, wait=4.0, save_dir=None, first_frame_timeout=20.0):
        self.url = url
        self.zone = zone
        self.dwell = dwell
        self.conf = conf
        self.nms = nms
        self.overlap = overlap
        self.grace = grace
        self.input_size = input_size
        self.models_dir = models_dir
        self.on_change = on_change
        self.on_log = on_log or (lambda msg: None)
        self.attempts = attempts
        self.wait = wait
        self.save_dir = save_dir
        self.first_frame_timeout = first_frame_timeout

    def _emit(self, active):
        if self.on_change is None:
            return
        try:
            self.on_change(bool(active))
        except Exception as e:
            self.on_log(f"on_change callback error: {e}")

    def _save_evidence(self, frame, dets, zone_px, dwell):
        if not self.save_dir:
            return
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            f = frame.copy()
            zx1, zy1, zx2, zy2 = zone_px
            cv2.rectangle(f, (zx1, zy1), (zx2, zy2), (0, 255, 255), 2)
            for (x1, y1, x2, y2, cf) in dets:
                in_z = overlap_ratio((x1, y1, x2, y2), zone_px) >= self.overlap
                col = (0, 0, 255) if in_z else (0, 255, 0)
                cv2.rectangle(f, (x1, y1), (x2, y2), col, 2)
            cv2.putText(f, time.strftime("%Y-%m-%d %H:%M:%S") +
                        f"  HSE ALARM ({dwell:.1f}s)", (8, 24), FONT, 0.7,
                        (0, 0, 255), 2)
            fn = os.path.join(self.save_dir, time.strftime("hse_%Y%m%d_%H%M%S.jpg"))
            cv2.imwrite(fn, f)
            self.on_log(f"evidence saved -> {fn}")
        except Exception as e:
            self.on_log(f"evidence save failed: {e}")

    def run(self, stop_event):
        """Blocking detection loop; returns when stop_event is set or the stream
        dies unrecoverably. Intended to be the target of a daemon thread."""
        cfg, wts = ensure_model(self.models_dir)          # may raise RuntimeError
        self.on_log(f"loading YOLOv4-tiny (cv2.dnn, CPU, input {self.input_size})")
        model = load_model(cfg, wts, self.input_size)

        grab = FrameGrabber(self.url, max(1, self.attempts), self.wait, stop_event)
        grab.start()

        # Wait (bounded) for the first real frame.
        t_wait = time.time()
        frame = None
        while not stop_event.is_set():
            _, frame = grab.read()
            if frame is not None:
                break
            if not grab.is_alive() or time.time() - t_wait > self.first_frame_timeout:
                grab.stop()
                raise RuntimeError("no video -- check RTSP enabled, port 554, IP, code")
            time.sleep(0.1)
        if frame is None:                                  # stopped before first frame
            grab.stop()
            return

        h, w = frame.shape[:2]
        zone_px = zone_to_px(self.zone, w, h)
        self.on_log(f"armed -- frame {w}x{h}, zone px {zone_px}, dwell {self.dwell:.1f}s")

        # Dwell state machine (all times wall-clock).
        t0 = None            # when the current continuous presence started
        last_seen = 0.0      # last time a person was in the zone
        alarmed = False      # has the alarm already fired for this presence
        last_seq = -1
        frames = 0           # frames processed (for the heartbeat)
        last_beat = time.time()

        try:
            while not stop_event.is_set():
                seq, frame = grab.read()
                if frame is None or seq == last_seq:
                    if not grab.is_alive():
                        raise RuntimeError("capture thread died")
                    if stop_event.wait(0.01):
                        break
                    continue
                last_seq = seq
                frames += 1

                dets, n_raw = detect_persons(model, frame, self.conf, self.nms)
                in_zone = [d for d in dets
                           if overlap_ratio(d, zone_px) >= self.overlap]
                now = time.time()

                # First few frames: show the raw model output immediately so you
                # can confirm the net produces detections (vs. seeing nothing).
                if frames <= 5:
                    self.on_log(f"frame {frames}: model raw boxes={n_raw}, "
                                f"persons={len(dets)}, in-zone={len(in_zone)}")

                if in_zone:
                    last_seen = now
                    if t0 is None:
                        t0 = now
                        self.on_log(f"person ENTERED zone ({len(in_zone)}, top conf "
                                    f"{max(d[4] for d in in_zone):.2f}) -- dwell timer started")
                    dwell = now - t0
                    if dwell >= self.dwell and not alarmed:
                        alarmed = True
                        self.on_log(f"*** HSE ALARM *** person in zone {dwell:.1f}s "
                                    f"(>= {self.dwell:.1f}s)")
                        self._emit(True)
                        self._save_evidence(frame, dets, zone_px, dwell)
                else:
                    # No one in the zone this frame; tolerate brief gaps via grace.
                    if t0 is not None and (now - last_seen) > self.grace:
                        if alarmed:
                            self.on_log("zone clear -- HSE alarm reset")
                            self._emit(False)
                        t0 = None
                        alarmed = False

                # Heartbeat every ~10s so the operator can confirm the detector is
                # alive and SEE what it sees (frames, persons, dwell, alarm state).
                if now - last_beat >= 10.0:
                    last_beat = now
                    self.on_log(f"alive: {frames} frames, raw boxes={n_raw}, "
                                f"{len(dets)} person(s), {len(in_zone)} in-zone, "
                                f"dwell={('%.1fs' % (now - t0)) if t0 else '-'}, "
                                f"alarm={alarmed}")
        finally:
            grab.stop()
            # If we exit while still alarmed, make sure the consumer clears it.
            if alarmed:
                self._emit(False)
