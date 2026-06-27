#!/usr/bin/env python3
"""
EZVIZ H8c live stream viewer (RTSP + OpenCV) with YOLO person detection.

RTSP must be enabled in the EZVIZ app first:
    Device Settings -> (camera) -> ... -> Local Service / LAN Live View -> RTSP

URL format for EZVIZ (Hikvision-based):
    rtsp://admin:<VERIFICATION_CODE>@<IP>:554/H264/ch1/main/av_stream   (main, HD)
    rtsp://admin:<VERIFICATION_CODE>@<IP>:554/H264/ch1/sub/av_stream    (sub, SD)

  * username is always  admin
  * password is the 6-char UPPERCASE verification/encryption code printed on the
    camera label (and shown in the app). NOT your EZVIZ account password.

Set the code via env var so it is not hard-coded:
    PowerShell:  $env:EZVIZ_CODE="ABCDEF"
    bash:        export EZVIZ_CODE=ABCDEF

Install (one-time):
    pip install ultralytics opencv-python
    # ultralytics pulls in torch; first run auto-downloads the model weights.

Run:
    python stream.py                 # main stream, person detection on
    python stream.py --sub           # lower-res sub stream (smoother)
    python stream.py --ip 192.168.100.29 --code ABCDEF
    python stream.py --no-detect     # disable YOLO, raw stream only
    python stream.py --model yolov8s.pt --conf 0.4
Press 'q' to quit, 's' to save a snapshot.
"""

import argparse
import os
import sys

import cv2

# Force FFmpeg to use TCP transport — far more reliable than UDP over WiFi.
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")

# COCO class id for "person".
PERSON_CLASS_ID = 0


def build_url(ip, code, sub):
    channel = "sub" if sub else "main"
    return f"rtsp://admin:{code}@{ip}:554/H264/ch1/{channel}/av_stream"


def load_model(model_name):
    """Lazy-load the YOLO model. Weights auto-download on first use."""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[error] ultralytics not installed. Run: pip install ultralytics",
              file=sys.stderr)
        sys.exit(1)
    print(f"Loading YOLO model '{model_name}' (first run downloads weights)...")
    return YOLO(model_name)


def detect_people(model, frame, conf):
    """Run YOLO on one frame, draw boxes around people, return (frame, count)."""
    # classes=[0] restricts inference to the 'person' class only.
    results = model.predict(frame, conf=conf, classes=[PERSON_CLASS_ID],
                            verbose=False)
    count = 0
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            score = float(box.conf[0])
            count += 1
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"person {score:.2f}"
            cv2.putText(frame, label, (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, f"People: {count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    return frame, count


def main():
    ap = argparse.ArgumentParser(description="EZVIZ H8c RTSP viewer + YOLO person detection")
    ap.add_argument("--ip", default="192.168.4.7", help="camera IP")
    ap.add_argument("--code", default=os.environ.get("EZVIZ_CODE", "Aman2026"),
                    help="camera RTSP password / verification code "
                         "(or set EZVIZ_CODE env var)")
    ap.add_argument("--sub", action="store_true", help="use sub (low-res) stream")
    ap.add_argument("--no-detect", action="store_true",
                    help="disable YOLO detection (raw stream only)")
    ap.add_argument("--model", default="yolov8n.pt",
                    help="YOLO weights (n/s/m/l/x; n=fastest, default yolov8n.pt)")
    ap.add_argument("--conf", type=float, default=0.35,
                    help="detection confidence threshold (0-1)")
    args = ap.parse_args()

    if not args.code:
        print("[error] no verification code. Use --code ABCDEF or set EZVIZ_CODE.",
              file=sys.stderr)
        sys.exit(1)

    model = None if args.no_detect else load_model(args.model)

    url = build_url(args.ip, args.code, args.sub)
    print(f"Opening {url.replace(args.code, '******')}")

    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[error] cannot open stream. Check: RTSP enabled in app, IP, code, "
              "same network.", file=sys.stderr)
        sys.exit(1)

    snap = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[warn] dropped frame / stream stalled")
            continue

        if model is not None:
            frame, _ = detect_people(model, frame, args.conf)

        cv2.imshow("EZVIZ H8c", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            fn = f"snapshot_{snap}.jpg"
            cv2.imwrite(fn, frame)
            print(f"saved {fn}")
            snap += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
