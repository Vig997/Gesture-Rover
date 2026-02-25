import argparse
import time

import cv2


DEFAULT_HOST = "172.20.10.5"  # Keep aligned with gesture_to_esp.py default.
DEFAULT_PORT = 12345          # Keep aligned with wifi_connect.ino port.
DEFAULT_PATH = "/stream"
MIN_DISPLAY_SCALE = 0.25
MAX_DISPLAY_SCALE = 1.0


def build_stream_url(host: str, port: int, path: str) -> str:
    normalized_path = path if path.startswith("/") else f"/{path}"
    return f"http://{host}:{port}{normalized_path}"


def open_stream(url: str) -> cv2.VideoCapture | None:
    for backend in (cv2.CAP_FFMPEG, cv2.CAP_ANY):
        cap = cv2.VideoCapture(url, backend)
        if cap.isOpened():
            return cap
        cap.release()
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"ESP32-CAM IP (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"HTTP port (default: {DEFAULT_PORT})")
    parser.add_argument("--path", default=DEFAULT_PATH, help=f"MJPEG path (default: {DEFAULT_PATH})")
    parser.add_argument(
        "--url",
        default="",
        help="Full stream URL override (e.g. http://172.20.10.6:12345/stream)",
    )
    parser.add_argument(
        "--reconnect-ms",
        type=int,
        default=500,
        help="Reconnect delay when stream drops (default: 500)",
    )
    parser.add_argument(
        "--target-fps",
        type=float,
        default=15.0,
        help="Viewer adaptation target FPS (default: 15.0)",
    )
    args = parser.parse_args()

    stream_url = args.url or build_stream_url(args.host, args.port, args.path)
    print(f"Opening stream: {stream_url}")

    cap = open_stream(stream_url)
    if cap is None:
        raise SystemExit(f"Failed to open stream at {stream_url}")

    last_fps_ts = time.time()
    frame_counter = 0
    fps = 0.0
    display_scale = 1.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Stream read failed; reconnecting...")
                cap.release()
                time.sleep(max(args.reconnect_ms, 0) / 1000.0)
                cap = open_stream(stream_url)
                if cap is None:
                    continue
                continue

            frame_counter += 1
            now = time.time()
            dt = now - last_fps_ts
            if dt >= 1.0:
                fps = frame_counter / dt
                frame_counter = 0
                last_fps_ts = now

                if fps < (args.target_fps - 1.0):
                    display_scale = max(MIN_DISPLAY_SCALE, display_scale * 0.85)
                elif fps > (args.target_fps + 4.0):
                    display_scale = min(MAX_DISPLAY_SCALE, display_scale * 1.05)

            if display_scale < 0.999:
                frame = cv2.resize(
                    frame,
                    (0, 0),
                    fx=display_scale,
                    fy=display_scale,
                    interpolation=cv2.INTER_AREA,
                )

            cv2.putText(
                frame,
                f"{stream_url} | {fps:.1f} FPS | scale {display_scale:.2f} | q quit",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("ESP32-CAM FPV", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
