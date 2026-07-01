#!/usr/bin/env python3
"""
EZVIZ H8c connectivity check for Raspberry Pi — STEP 1: just get the picture.

Connects to the RTSP stream, confirms frames actually arrive, reports
resolution + measured FPS, and saves a snapshot JPG as visual proof. Works
headless (over SSH, no display). No YOLO / no detection yet — that is step 2,
which slots into the marked hook in the frame loop.

RTSP must be enabled on the camera (encryption OFF in the EZVIZ app). After a
camera reboot the RTSP service takes ~30-60s to come up, so this script retries
the connection a few times before giving up.

URL format for EZVIZ (Hikvision-based):
    rtsp://admin:<CODE>@<IP>:554/H264/ch1/main/av_stream   (main, HD)
    rtsp://admin:<CODE>@<IP>:554/H264/ch1/sub/av_stream    (sub, SD — default)

  * username is always  admin
  * password is the verification/encryption code on the camera label / app.
    NOT your EZVIZ account password.

Set the code via env var so it is not hard-coded:
    export EZVIZ_CODE=NANXJW

Install on the Pi (one-time):
    sudo apt install -y python3-opencv libgl1

Run:
    python3 rpi_camera_check.py                 # sub stream, save snapshot, print stats
    python3 rpi_camera_check.py --main          # full-res main stream
    python3 rpi_camera_check.py --seconds 8     # sample longer before reporting
    python3 rpi_camera_check.py --show          # live preview window (needs a display)
    python3 rpi_camera_check.py --record 5      # also save a 5s .avi clip
    python3 rpi_camera_check.py --ip 192.168.100.13 --code NANXJW
Stop a live preview with 'q'.
"""

import argparse
import os
import sys
import time

import cv2

# Force FFmpeg to use TCP (reliable over WiFi) and fail fast if the socket
# stalls, instead of hanging the default 30s. stimeout is in microseconds;
# builds that don't recognise it simply ignore it (no harm).
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;5000000")


def build_url(ip, code, sub):
    channel = "sub" if sub else "main"
    return f"rtsp://admin:{code}@{ip}:554/H264/ch1/{channel}/av_stream"


def open_stream(url, attempts, wait):
    """Open the RTSP stream, retrying — the camera's RTSP daemon boots slowly."""
    for i in range(1, attempts + 1):
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            return cap
        cap.release()
        if i < attempts:
            print(f"[warn] open failed (attempt {i}/{attempts}); "
                  f"retrying in {wait}s — camera may still be booting")
            time.sleep(wait)
    return None


def run_check(cap, seconds, snapshot, show, record):
    """Pull frames for `seconds`, prove the picture, save a snapshot.

    Returns True if at least one real frame was decoded.
    """
    # Report what the camera says it is sending.
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Stream opened. Reported: {w}x{h} @ {reported_fps:.1f} fps "
          f"(reported values can be unreliable on RTSP)")

    writer = None
    frames = 0
    last_frame = None
    start = time.time()
    first_frame_deadline = start + 10.0  # allow warmup before declaring failure

    print(f"Sampling frames for {seconds}s... (Ctrl-C to stop early)")
    try:
        while time.time() - start < seconds:
            ok, frame = cap.read()
            if not ok or frame is None:
                # RTSP often yields a few empty reads while it warms up.
                if frames == 0 and time.time() < first_frame_deadline:
                    time.sleep(0.05)
                    continue
                print("[warn] dropped frame / stream stalled")
                time.sleep(0.05)
                continue

            frames += 1
            last_frame = frame

            # ---- STEP 2 HOOK: run people detection on `frame` here ----
            # e.g. count = count_people(model, frame, conf); annotate frame.

            if record:
                if writer is None:
                    fh, fw = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                    writer = cv2.VideoWriter("camera_check.avi", fourcc, 15.0,
                                             (fw, fh))
                writer.write(frame)

            if show:
                try:
                    cv2.imshow("EZVIZ camera check", frame)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break
                except cv2.error:
                    print("[warn] --show needs a display; disabling preview")
                    show = False
    except KeyboardInterrupt:
        print("\nstopped early by user")
    finally:
        if writer is not None:
            writer.release()
            print("saved clip -> camera_check.avi")
        if show:
            cv2.destroyAllWindows()

    elapsed = max(1e-3, time.time() - start)
    if frames == 0 or last_frame is None:
        print("[error] connected but decoded 0 frames — stream opened but no "
              "video. Check codec (H264 sub/main) and that the camera is live.")
        return False

    fh, fw = last_frame.shape[:2]
    print(f"OK — decoded {frames} frames in {elapsed:.1f}s "
          f"(~{frames / elapsed:.1f} fps effective), frame size {fw}x{fh}")

    if cv2.imwrite(snapshot, last_frame):
        print(f"snapshot saved -> {snapshot}  (open it to confirm the picture)")
    else:
        print(f"[warn] could not write snapshot to {snapshot}")
    return True


def main():
    ap = argparse.ArgumentParser(
        description="EZVIZ H8c connectivity check (Raspberry Pi, step 1)")
    ap.add_argument("--ip", default="192.168.100.13", help="camera IP")
    ap.add_argument("--code", default=os.environ.get("EZVIZ_CODE", "NANXJW"),
                    help="RTSP password / verification code (or set EZVIZ_CODE)")
    ap.add_argument("--main", action="store_true",
                    help="use main (HD) stream; default is sub (low-res, lighter)")
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="how long to sample frames before reporting (default 5)")
    ap.add_argument("--snapshot", default="camera_check.jpg",
                    help="path to save the proof snapshot")
    ap.add_argument("--show", action="store_true",
                    help="show a live preview window (needs a display / VNC)")
    ap.add_argument("--record", type=float, default=0,
                    help="also save an N-second .avi clip (0 = off)")
    ap.add_argument("--attempts", type=int, default=5,
                    help="RTSP open retries (camera boots slowly after reboot)")
    ap.add_argument("--wait", type=float, default=4.0,
                    help="seconds between open retries")
    args = ap.parse_args()

    if not args.code:
        print("[error] no verification code. Use --code XXXX or set EZVIZ_CODE.",
              file=sys.stderr)
        sys.exit(1)

    use_sub = not args.main
    url = build_url(args.ip, args.code, use_sub)
    print(f"Opening {url.replace(args.code, '******')}")

    cap = open_stream(url, max(1, args.attempts), args.wait)
    if cap is None:
        print("[error] cannot open stream after retries. Check: RTSP enabled "
              "(encryption OFF), port 554 reachable, IP, code, same network.",
              file=sys.stderr)
        sys.exit(1)

    # --record N implies sampling for at least N seconds.
    seconds = max(args.seconds, args.record)
    try:
        ok = run_check(cap, seconds, args.snapshot, args.show, args.record > 0)
    finally:
        cap.release()

    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
