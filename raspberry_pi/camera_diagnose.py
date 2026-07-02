#!/usr/bin/env python3
"""
camera_diagnose.py -- two-step camera + YOLO diagnostic for the HSE alarm.

STEP 1  CAPTURE: records --seconds (default 10) of the SAME RTSP stream
        monitoring_control.py uses and saves it to ONE video file
        (capture_YYYYmmdd_HHMMSS.avi, MJPG) plus start/middle/end JPGs.
        Copy the .avi to a PC and watch it -- this is EXACTLY what the
        detector sees (aim, brightness, person visible or not).

STEP 2  MODEL CHECK: runs YOLOv4-tiny on ~20 frames sampled from the capture
        and prints EVERY object it finds (all 80 COCO classes, not only
        person) at conf 0.50 and again at a relaxed 0.25. Annotated JPGs of
        the sampled frames go to diagnose_out/. If the model sees chairs/tv
        but no person while a person WAS in view -> visibility problem (aim,
        distance, glass, backlight). If it sees nothing at all in a normal
        scene -> model/pipeline problem.

!! STOP monitoring_control.py FIRST -- the EZVIZ camera may allow only one
   RTSP client, so this script cannot open the stream while it runs.

Run on the Pi from the raspberry_pi/ folder:
    python3 camera_diagnose.py                      # 10 s capture + model check
    python3 camera_diagnose.py --seconds 15 --main  # HD stream, 15 s
    python3 camera_diagnose.py --no-model           # capture only
Stand INSIDE the camera view while it records.
"""

import argparse
import os
import sys
import time

# Force FFmpeg RTSP over TCP + fail fast on a stalled socket (same as
# monitoring_control.py). Must be set before cv2 is imported.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;5000000")

import cv2          # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# COCO class names in darknet order (what yolov4-tiny.weights was trained on).
COCO = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant",
    "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


def build_rtsp_url(ip, code, sub):
    channel = "sub" if sub else "main"
    return f"rtsp://admin:{code}@{ip}:554/H264/ch1/{channel}/av_stream"


def open_capture(url, attempts=5, wait=4.0):
    for i in range(1, attempts + 1):
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            return cap
        cap.release()
        if i < attempts:
            print(f"[capture] open failed ({i}/{attempts}); retrying in "
                  f"{wait:.0f}s -- if monitoring_control.py is running, STOP "
                  f"it (one RTSP client only)", flush=True)
            time.sleep(wait)
    return None


# ------------------------------------------------------------------ STEP 1 ---
def record(args):
    """Capture --seconds of video into one MJPG .avi + 3 snapshot JPGs.
    Returns (video_path, sampled_frames) -- ~20 frames kept for the model."""
    url = build_rtsp_url(args.ip, args.code, sub=not args.main)
    print(f"[capture] opening {url.replace(args.code, '******')}", flush=True)
    cap = open_capture(url)
    if cap is None:
        print("[capture] FAILED to open the stream -- check IP/code/port 554, "
              "and that no other client (monitoring_control.py, phone app, "
              "standalone script) is connected", flush=True)
        sys.exit(1)

    # First frame tells us the real size; fps from the stream when sane.
    ok, frame = cap.read()
    if not ok or frame is None:
        print("[capture] stream opened but no frame arrived", flush=True)
        sys.exit(1)
    h, w = frame.shape[:2]
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not (5 <= fps <= 60):
        fps = 20.0                       # EZVIZ sub-stream nominal
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(HERE, args.out or f"capture_{stamp}.avi")
    vw = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))
    if not vw.isOpened():
        print("[capture] VideoWriter failed to open (codec?)", flush=True)
        sys.exit(1)

    print(f"[capture] recording {args.seconds:.0f}s @ {w}x{h} (~{fps:.0f} fps "
          f"nominal) -> {out_path}", flush=True)
    print("[capture] STAND IN THE CAMERA VIEW NOW", flush=True)

    frames, fails = [], 0
    n = 0
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        ok, f = cap.read()
        if not ok or f is None:
            fails += 1
            time.sleep(0.02)
            continue
        vw.write(f)
        frames.append(f)
        n += 1
    dur = time.time() - t0
    vw.release()
    cap.release()

    if n == 0:
        print("[capture] no frames captured -- stream dead", flush=True)
        sys.exit(1)

    # snapshots + brightness report
    picks = {"start": frames[0], "middle": frames[len(frames) // 2],
             "end": frames[-1]}
    for name, f in picks.items():
        p = os.path.join(HERE, f"capture_{stamp}_{name}.jpg")
        cv2.imwrite(p, f)
        print(f"[capture] snapshot -> {p}", flush=True)
    bright = float(np.mean(cv2.cvtColor(frames[len(frames) // 2],
                                        cv2.COLOR_BGR2GRAY)))
    verdict = ("DARK -- detection will struggle" if bright < 40 else
               "bright" if bright > 180 else "ok")
    print(f"[capture] done: {n} frames in {dur:.1f}s = {n / dur:.1f} fps real, "
          f"{fails} failed reads, mid-frame brightness {bright:.0f}/255 "
          f"({verdict})", flush=True)
    print(f"[capture] WATCH THE VIDEO: {out_path}", flush=True)

    step = max(1, len(frames) // 20)     # ~20 frames for the model check
    return out_path, frames[::step]


# ------------------------------------------------------------------ STEP 2 ---
def model_check(args, samples):
    cfg = os.path.join(args.models_dir, "yolov4-tiny.cfg")
    wts = os.path.join(args.models_dir, "yolov4-tiny.weights")
    if not (os.path.exists(cfg) and os.path.exists(wts)):
        print(f"[model] MISSING model files in {args.models_dir} -- skipped",
              flush=True)
        return
    print(f"[model] loading YOLOv4-tiny (input {args.input})", flush=True)
    net = cv2.dnn.readNetFromDarknet(cfg, wts)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    model = cv2.dnn.DetectionModel(net)
    model.setInputParams(scale=1 / 255.0, size=(args.input, args.input),
                         swapRB=True)

    out_dir = os.path.join(HERE, "diagnose_out")
    os.makedirs(out_dir, exist_ok=True)
    font = cv2.FONT_HERSHEY_SIMPLEX

    for conf in (0.50, 0.25):
        print(f"\n[model] ---- pass at conf {conf:.2f} on {len(samples)} "
              f"sampled frames ----", flush=True)
        persons_total, objects_total = 0, {}
        for i, f in enumerate(samples):
            ids, cfs, boxes = model.detect(f, confThreshold=conf,
                                           nmsThreshold=0.40)
            found = []
            if len(boxes):
                ids = np.array(ids).reshape(-1)
                cfs = np.array(cfs).reshape(-1)
                img = f.copy()
                for cid, cf, box in zip(ids, cfs, boxes):
                    name = COCO[int(cid)] if int(cid) < len(COCO) else str(cid)
                    found.append(f"{name} {cf:.2f}")
                    objects_total[name] = objects_total.get(name, 0) + 1
                    if name == "person":
                        persons_total += 1
                    x, y, bw, bh = box
                    col = (0, 0, 255) if name == "person" else (0, 255, 0)
                    cv2.rectangle(img, (x, y), (x + bw, y + bh), col, 2)
                    cv2.putText(img, f"{name} {cf:.2f}", (x, max(12, y - 6)),
                                font, 0.5, col, 2)
                if conf == 0.50:          # annotate once, from the strict pass
                    p = os.path.join(out_dir, f"frame{i:02d}_annotated.jpg")
                    cv2.imwrite(p, img)
            print(f"[model] frame {i:02d}: "
                  f"{', '.join(found) if found else '(nothing)'}", flush=True)
        print(f"[model] conf {conf:.2f} SUMMARY: person hits in "
              f"{persons_total}/{len(samples)} frames | all objects: "
              f"{objects_total or 'NONE'}", flush=True)

    print(f"\n[model] annotated frames (conf 0.50) -> {out_dir}", flush=True)
    print("[model] read it like this:", flush=True)
    print("  * person seen here but not by monitoring_control -> zone/config "
          "issue", flush=True)
    print("  * other objects seen but person missed while in view -> "
          "visibility (aim/distance/glass/backlight)", flush=True)
    print("  * NOTHING seen at any conf in a normal scene -> model/pipeline "
          "problem", flush=True)


def main():
    ap = argparse.ArgumentParser(
        description="Capture N seconds from the EZVIZ camera to a file, then "
                    "run YOLOv4-tiny on the captured frames")
    ap.add_argument("--ip", default="192.168.100.13", help="camera IP")
    ap.add_argument("--code", default=os.environ.get("EZVIZ_CODE", "NANXJW"),
                    help="RTSP verification code (or set EZVIZ_CODE)")
    ap.add_argument("--main", action="store_true",
                    help="use main (HD) stream; default sub (what the "
                         "detector uses)")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="capture length (default 10)")
    ap.add_argument("--out", default=None,
                    help="output video filename (default capture_<stamp>.avi)")
    ap.add_argument("--models-dir",
                    default=os.path.join(os.path.dirname(HERE),
                                         "ezviz_camera", "models"),
                    help="dir with yolov4-tiny.cfg/.weights")
    ap.add_argument("--input", type=int, default=416,
                    help="YOLO input size (must match monitoring_control)")
    ap.add_argument("--no-model", action="store_true",
                    help="capture only, skip the YOLO check")
    args = ap.parse_args()

    _, samples = record(args)
    if not args.no_model:
        model_check(args, samples)


if __name__ == "__main__":
    main()
