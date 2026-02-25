import argparse
import os
import socket
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np

from cv_to_packet import width_angle_to_packet
from steering_wheel_tracker import (
    angle_deg_from_centers,
    create_landmarker,
    draw_hand,
    ensure_model,
    grip_point_px,
    wrap_deg_180,
)


def connect_tcp(host: str, port: int, timeout_s: float) -> socket.socket | None:
    try:
        sock = socket.create_connection((host, port), timeout=timeout_s)
        sock.settimeout(timeout_s)
        return sock
    except OSError:
        return None


def main() -> None:
    default_model = os.path.join(os.path.dirname(__file__), "models", "hand_landmarker.task")

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--host",
        default="172.20.10.6",
        help="ESP32 IP/hostname (default: 172.20.10.6)",
    )
    ap.add_argument("--port", type=int, default=12345, help="ESP32 WiFiServer port (default: 12345)")
    ap.add_argument("--camera", type=int, default=0, help="Webcam index (default: 0)")
    ap.add_argument("--model", default=default_model, help="Path to hand_landmarker.task")
    ap.add_argument("--frame-width", type=int, default=640, help="Capture width (default: 640)")
    ap.add_argument("--frame-height", type=int, default=360, help="Capture height (default: 360)")
    ap.add_argument("--send-ms", type=int, default=20, help="Packet send period in ms (default: 20)")
    ap.add_argument(
        "--hold-ms",
        type=int,
        default=2000,
        help="Hold last valid packet this many ms when both hands are lost (default: 2000)",
    )
    ap.add_argument("--connect-timeout", type=float, default=1.0, help="TCP connect timeout seconds")
    ap.add_argument("--no-gui", action="store_true", help="Run without cv2.imshow")
    ap.add_argument("--self-test", action="store_true", help="Load model and exit")
    args = ap.parse_args()

    if os.environ.get("DISPLAY") is None:
        args.no_gui = True

    ensure_model(args.model)
    landmarker = create_landmarker(args.model, num_hands=2)

    if args.self_test:
        print("OK: model loaded")
        return

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open webcam (index {args.camera})")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.frame_width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.frame_height))

    smooth = deque(maxlen=7)
    dist_filter = deque(maxlen=7)
    prev_raw = None
    unwrap_offset = 0.0
    missing_frames = 0
    last_angle_out = 0.0
    last_angle_unwrapped = 0.0

    sock = None
    last_connect_try = 0.0
    last_connect_log = 0.0
    last_send = 0.0
    last_print = 0.0
    last_valid_packet = "00000000"
    last_valid_packet_ts = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts_ms = int(time.time() * 1000)
            result = landmarker.detect_for_video(mp_image, ts_ms)

            centers_by_label = {}
            if result.hand_landmarks and result.handedness:
                for idx, hand_lms in enumerate(result.hand_landmarks):
                    label = result.handedness[idx][0].category_name
                    score = float(result.handedness[idx][0].score)
                    center = grip_point_px(hand_lms, w, h)

                    if label == "Left":
                        label = "Right"
                    elif label == "Right":
                        label = "Left"

                    if label not in centers_by_label or score > centers_by_label[label][0]:
                        centers_by_label[label] = (score, center, hand_lms)

                    if not args.no_gui:
                        draw_hand(frame, hand_lms)

            width_ratio = 0.0
            angle_out = last_angle_out
            tracking_valid = False

            if "Left" in centers_by_label and "Right" in centers_by_label:
                tracking_valid = True
                missing_frames = 0
                left_c = centers_by_label["Left"][1]
                right_c = centers_by_label["Right"][1]

                dist_px_raw = float(np.hypot(right_c[0] - left_c[0], right_c[1] - left_c[1]))
                dist_filter.append(dist_px_raw)
                dist_px = float(np.mean(dist_filter))
                width_ratio = float(np.clip((2.0 * dist_px) / max(float(w), 1.0), 0.0, 2.0))

                raw = angle_deg_from_centers(left_c, right_c)
                if prev_raw is None and not smooth:
                    k = int(round((last_angle_unwrapped - raw) / 360.0))
                    unwrap_offset = 360.0 * k
                if prev_raw is not None:
                    diff = raw - prev_raw
                    if diff > 180.0:
                        unwrap_offset -= 360.0
                    elif diff < -180.0:
                        unwrap_offset += 360.0
                prev_raw = raw

                unwrapped = raw + unwrap_offset
                smooth.append(unwrapped)
                filtered = float(np.mean(smooth))
                last_angle_unwrapped = filtered
                angle_out = wrap_deg_180(filtered)
                last_angle_out = angle_out

                if not args.no_gui:
                    cv2.line(frame, left_c, right_c, (0, 255, 0), 4, cv2.LINE_AA)
                    cv2.circle(frame, left_c, 8, (255, 255, 255), -1, cv2.LINE_AA)
                    cv2.circle(frame, right_c, 8, (255, 255, 255), -1, cv2.LINE_AA)
            else:
                missing_frames += 1
                if missing_frames > 10:
                    smooth.clear()
                    dist_filter.clear()
                    prev_raw = None
                    unwrap_offset = 0.0
                width_ratio = 0.0
                angle_out = last_angle_out

            now = time.time()
            raw_packet = width_angle_to_packet(width_ratio, angle_out)
            hold_active = False
            if tracking_valid:
                packet = raw_packet
                last_valid_packet = raw_packet
                last_valid_packet_ts = now
            elif (
                last_valid_packet_ts is not None
                and (now - last_valid_packet_ts) * 1000.0 <= args.hold_ms
            ):
                packet = last_valid_packet
                hold_active = True
            else:
                packet = raw_packet

            if sock is None and (now - last_connect_try) >= 1.0:
                last_connect_try = now
                sock = connect_tcp(args.host, args.port, args.connect_timeout)
                if sock is not None:
                    print(f"Connected to {args.host}:{args.port}")
                elif (now - last_connect_log) >= 2.0:
                    print(f"Retrying connection to {args.host}:{args.port} ...")
                    last_connect_log = now

            if (now - last_send) * 1000.0 >= args.send_ms:
                if sock is not None:
                    try:
                        sock.sendall(packet.encode("ascii"))
                    except OSError:
                        try:
                            sock.close()
                        except OSError:
                            pass
                        sock = None
                last_send = now

            if args.no_gui:
                if (now - last_print) >= 0.1:
                    status = "CONNECTED" if sock is not None else "DISCONNECTED"
                    mode = "TRACK" if tracking_valid else ("HOLD" if hold_active else "BRAKE")
                    print(
                        f"status={status} mode={mode} width={width_ratio:.2f} angle={angle_out:.1f} packet={packet}"
                    )
                    last_print = now
                continue

            status = "CONNECTED" if sock is not None else "DISCONNECTED"
            mode = "TRACK" if tracking_valid else ("HOLD" if hold_active else "BRAKE")
            cv2.putText(
                frame,
                f"w {width_ratio:.2f}x | deg {angle_out:.1f} | pkt {packet} | {mode}",
                (10, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"ESP {args.host}:{args.port} [{status}] send {args.send_ms}ms | q quit",
                (10, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow("Gesture -> ESP Packets", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        try:
            if sock is not None:
                sock.close()
        except OSError:
            pass
        cap.release()
        if not args.no_gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
