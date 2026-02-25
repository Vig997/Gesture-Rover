import argparse
import time

import cv2
import numpy as np


DEFAULT_HOST = "172.20.10.5"  # Keep aligned with gesture_to_esp.py default.
DEFAULT_PORT = 12345          # Keep aligned with wifi_connect.ino port.
DEFAULT_PATH = "/stream"
MIN_DISPLAY_SCALE = 0.25
MAX_DISPLAY_SCALE = 1.0
STATUS_BAR_HEIGHT = 36
DEFAULT_WINDOW_WIDTH = 960
DEFAULT_WINDOW_HEIGHT = 540


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


def fit_status_text(text: str, max_width: int, font, font_scale: float, thickness: int) -> str:
    if max_width <= 20:
        return ""
    rendered, _ = cv2.getTextSize(text, font, font_scale, thickness)
    if rendered[0] <= max_width:
        return text

    ellipsis = "..."
    lo = 0
    hi = len(text)
    best = ellipsis
    while lo <= hi:
        mid = (lo + hi) // 2
        candidate = text[:mid] + ellipsis
        size, _ = cv2.getTextSize(candidate, font, font_scale, thickness)
        if size[0] <= max_width:
            best = candidate
            lo = mid + 1
        else:
            hi = mid - 1
    return best


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
    parser.add_argument(
        "--window-width",
        type=int,
        default=DEFAULT_WINDOW_WIDTH,
        help=f"Display width in pixels (default: {DEFAULT_WINDOW_WIDTH})",
    )
    parser.add_argument(
        "--window-height",
        type=int,
        default=DEFAULT_WINDOW_HEIGHT,
        help=f"Display height in pixels (default: {DEFAULT_WINDOW_HEIGHT})",
    )
    args = parser.parse_args()

    stream_url = args.url or build_stream_url(args.host, args.port, args.path)
    print(f"Opening stream: {stream_url}")

    cap = open_stream(stream_url)
    if cap is None:
        raise SystemExit(f"Failed to open stream at {stream_url}")

    window_name = "ESP32-CAM FPV"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    last_fps_ts = time.time()
    frame_counter = 0
    fps = 0.0
    display_scale = 1.0
    target_w = max(args.window_width, 320)
    target_h = max(args.window_height, 240)
    cv2.resizeWindow(window_name, target_w, target_h + STATUS_BAR_HEIGHT)

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

            src_h, src_w = frame.shape[:2]
            fit_scale = min(target_w / max(src_w, 1), target_h / max(src_h, 1))
            fit_w = max(1, int(src_w * fit_scale))
            fit_h = max(1, int(src_h * fit_scale))
            resized = cv2.resize(frame, (fit_w, fit_h), interpolation=cv2.INTER_LINEAR)

            frame_canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
            x0 = (target_w - fit_w) // 2
            y0 = (target_h - fit_h) // 2
            frame_canvas[y0 : y0 + fit_h, x0 : x0 + fit_w] = resized

            display_frame = cv2.copyMakeBorder(
                frame_canvas,
                STATUS_BAR_HEIGHT,
                0,
                0,
                0,
                cv2.BORDER_CONSTANT,
                value=(0, 0, 0),
            )

            status = f"{args.host}:{args.port}{args.path} | {fps:.1f} FPS | scale {display_scale:.2f} | q quit"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.50
            thickness = 1
            status = fit_status_text(status, target_w - 20, font, font_scale, thickness)
            cv2.putText(
                display_frame,
                status,
                (10, 24),
                font,
                font_scale,
                (0, 255, 0),
                thickness,
                cv2.LINE_AA,
            )
            cv2.imshow(window_name, display_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
