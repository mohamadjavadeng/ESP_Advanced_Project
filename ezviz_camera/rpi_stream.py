#!/usr/bin/env python3
"""
EZVIZ H8c headless person detector for Raspberry Pi 4 (no display / no GUI).

Reads the RTSP stream, runs YOLO person detection, and PRINTS when people are
detected. No frames are shown (no OpenCV HighGUI needed) — safe over SSH.

RTSP must be enabled in the EZVIZ app first:
    Device Settings -> (camera) -> ... -> Local Service / LAN Live View -> RTSP

URL format for EZVIZ (Hikvision-based):
    rtsp://admin:<VERIFICATION_CODE>@<IP>:554/H264/ch1/main/av_stream   (main, HD)
    rtsp://admin:<VERIFICATION_CODE>@<IP>:554/H264/ch1/sub/av_stream    (sub, SD)

  * username is always  admin
  * password is the verification/encryption code printed on the camera label
    (and shown in the app). NOT your EZVIZ account password.

Set the code via env var so it is not hard-coded:
    export EZVIZ_CODE=Aman2026

Install on the Pi (one-time):
    sudo apt install -y python3-opencv libgl1
    pip install ultralytics
    # On Pi 4 (CPU only) use the sub stream + frame skipping for usable speed.

Run:
    python3 rpi_stream.py                  # sub stream, person detection, headless
    python3 rpi_stream.py --main           # full-res main stream (slower on Pi)
    python3 rpi_stream.py --every 5        # run YOLO on every 5th frame (default 3)
    python3 rpi_stream.py --conf 0.4
Stop with Ctrl-C.
"""

import argparse
import os
import sys
import time

# --- Raspberry Pi "Illegal instruction" (SIGILL) guards -----------------------
# Must be set BEFORE numpy / cv2 / torch are imported, or they have no effect.
# Pi 4 = Cortex-A72 (ARMv8.0). Some BLAS builds auto-detect a newer core and
# emit ARMv8.2 instructions (dotprod/fp16) the A72 lacks -> SIGILL inside the
# numpy/predict math. Force the generic ARMv8 kernel path.
os.environ.setdefault("OPENBLAS_CORETYPE", "ARMV8")
# Keep BLAS single-threaded; oversubscription on the Pi can also trip bad kernels
# and just adds contention with 4 cores busy decoding video.
os.environ.setdefault("OMP_NUM_THREADS", "2")

import cv2

# Force FFmpeg to use TCP transport — far more reliable than UDP over WiFi.
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")

# COCO class id for "person".
PERSON_CLASS_ID = 0


def build_url(ip, code, sub):
    channel = "sub" if sub else "main"
    return f"rtsp://admin:{code}@{ip}:554/H264/ch1/{channel}/av_stream"


def load_model(model_name):
    """Lazy-load the YOLO model.

    On the Pi, prefer an NCNN model directory (e.g. 'yolov8n_ncnn_model') —
    NCNN runs the network with NEON kernels and NO PyTorch, which avoids the
    'Illegal instruction' (SIGILL) crash that torch's CPU kernels trigger on the
    Cortex-A72, and is ~2-3x faster. A plain '.pt' falls back to the torch
    backend (fine on a PC, but the source of the SIGILL on the Pi).
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[error] ultralytics not installed. Run: pip install ultralytics",
              file=sys.stderr)
        sys.exit(1)

    is_ncnn = os.path.isdir(model_name) or model_name.endswith("_ncnn_model")
    if is_ncnn and not os.path.isdir(model_name):
        print(f"[error] NCNN model dir '{model_name}' not found. Export it on a "
              f"machine where torch works:\n"
              f"    yolo export model=yolov8n.pt format=ncnn\n"
              f"then copy the '{model_name}' folder next to this script.",
              file=sys.stderr)
        sys.exit(1)
    if is_ncnn:
        try:
            import ncnn  # noqa: F401
        except ImportError:
            print("[error] ncnn runtime not installed. Run: pip install ncnn",
                  file=sys.stderr)
            sys.exit(1)

    print(f"Loading YOLO model '{model_name}'"
          f"{' (NCNN backend)' if is_ncnn else ''} ...")
    # task='detect' avoids a guess-warning when loading an NCNN dir.
    return YOLO(model_name, task="detect")


def count_people(model, frame, conf):
    """Run YOLO on one frame, return number of people detected."""
    # classes=[0] restricts inference to the 'person' class only.
    results = model.predict(frame, conf=conf, classes=[PERSON_CLASS_ID],
                            verbose=False)
    count = 0
    for r in results:
        count += len(r.boxes)
    return count


def main():
    ap = argparse.ArgumentParser(
        description="EZVIZ H8c headless person detector (Raspberry Pi)")
    ap.add_argument("--ip", default="192.168.100.29", help="camera IP")
    ap.add_argument("--code", default=os.environ.get("EZVIZ_CODE", "Aman2026"),
                    help="camera RTSP password / verification code "
                         "(or set EZVIZ_CODE env var)")
    ap.add_argument("--main", action="store_true",
                    help="use main (HD) stream; default is sub (low-res, faster)")
    ap.add_argument("--model", default="yolov8n_ncnn_model",
                    help="model to load. Default = NCNN dir 'yolov8n_ncnn_model' "
                         "(no torch, no SIGILL on Pi). Pass 'yolov8n.pt' to use "
                         "the torch backend instead.")
    ap.add_argument("--conf", type=float, default=0.35,
                    help="detection confidence threshold (0-1)")
    ap.add_argument("--every", type=int, default=3,
                    help="run YOLO on every Nth frame (higher = faster, default 3)")
    args = ap.parse_args()

    if not args.code:
        print("[error] no verification code. Use --code XXXX or set EZVIZ_CODE.",
              file=sys.stderr)
        sys.exit(1)

    model = load_model(args.model)

    use_sub = not args.main
    url = build_url(args.ip, args.code, use_sub)
    print(f"Opening {url.replace(args.code, '******')}")

    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[error] cannot open stream. Check: RTSP enabled in app, IP, code, "
              "same network.", file=sys.stderr)
        sys.exit(1)

    every = max(1, args.every)
    frame_i = 0
    present = False          # track person-present state to avoid spamming prints
    print("Detecting... (Ctrl-C to stop)")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[warn] dropped frame / stream stalled")
                time.sleep(0.05)
                continue

            frame_i += 1
            if frame_i % every != 0:
                continue

            count = count_people(model, frame, args.conf)
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            if count > 0:
                # print every detection cycle while someone is in view
                print(f"[{ts}] PERSON DETECTED — {count} person(s)")
                present = True
            elif present:
                # transition: people left the frame
                print(f"[{ts}] area clear (no person)")
                present = False
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
