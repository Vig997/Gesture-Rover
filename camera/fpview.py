import argparse
import socket
import struct
import time

import cv2
import numpy as np


DEFAULT_HOST = "172.20.10.5"
DEFAULT_PORT = 12345
DEFAULT_BIND = "0.0.0.0"
MIN_DISPLAY_SCALE = 0.25
MAX_DISPLAY_SCALE = 1.0
STATUS_BAR_HEIGHT = 36
DEFAULT_WINDOW_WIDTH = 960
DEFAULT_WINDOW_HEIGHT = 540
PROTO_MAGIC = b"UF"
HEADER_FMT = ">2sIHHH"  # magic, frame_id, chunk_idx, total_chunks, payload_len
HEADER_SIZE = struct.calcsize(HEADER_FMT)
STATS_INTERVAL_S = 1.0


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


def decode_complete_frame(frame_entry: dict) -> np.ndarray | None:
    total = frame_entry["total"]
    chunks = frame_entry["chunks"]
    for index in range(total):
        if index not in chunks:
            return None
    jpeg = b"".join(chunks[index] for index in range(total))
    frame_buf = np.frombuffer(jpeg, dtype=np.uint8)
    if frame_buf.size == 0:
        return None
    return cv2.imdecode(frame_buf, cv2.IMREAD_COLOR)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"ESP32-CAM IP (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"ESP32 UDP port (default: {DEFAULT_PORT})")
    parser.add_argument("--bind", default=DEFAULT_BIND, help=f"Local bind IP (default: {DEFAULT_BIND})")
    parser.add_argument("--local-port", type=int, default=DEFAULT_PORT, help=f"Local UDP port (default: {DEFAULT_PORT})")
    parser.add_argument("--target-fps", type=float, default=15.0, help="Viewer adaptation target FPS (default: 15.0)")
    parser.add_argument("--hello-ms", type=int, default=500, help="HELLO/KEEPALIVE period in ms (default: 500)")
    parser.add_argument("--stale-ms", type=int, default=800, help="Drop incomplete frame buffers older than this (default: 800)")
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

    esp_addr = (args.host, args.port)
    print(f"Listening UDP on {args.bind}:{args.local_port}, requesting stream from {args.host}:{args.port}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
    sock.bind((args.bind, args.local_port))
    sock.settimeout(0.02)

    window_name = "ESP32-CAM FPV (UDP)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    target_w = max(args.window_width, 320)
    target_h = max(args.window_height, 240)
    cv2.resizeWindow(window_name, target_w, target_h + STATUS_BAR_HEIGHT)

    pending_frames: dict[int, dict] = {}
    latest_frame: np.ndarray | None = None
    latest_frame_id = -1

    display_scale = 1.0
    rx_fps = 0.0
    decoded_frames = 0
    decoded_frames_prev = 0
    bad_packets = 0
    dropped_frames = 0
    last_stats_ts = time.time()
    last_hello_ts = 0.0
    last_rx_ts = 0.0
    hello_interval = max(args.hello_ms, 100) / 1000.0
    stale_timeout = max(args.stale_ms, 100) / 1000.0
    ack_seen = False

    try:
        while True:
            now = time.time()

            if (now - last_hello_ts) >= hello_interval:
                payload = b"HELLO" if not ack_seen else b"KEEPALIVE"
                sock.sendto(payload, esp_addr)
                last_hello_ts = now

            try:
                data, src = sock.recvfrom(2048)
            except socket.timeout:
                data = b""
                src = None

            if data:
                last_rx_ts = now
                if src and src[0] != args.host:
                    bad_packets += 1
                elif data.startswith(b"ACK"):
                    ack_seen = True
                elif len(data) < HEADER_SIZE:
                    bad_packets += 1
                else:
                    magic, frame_id, chunk_idx, total_chunks, payload_len = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
                    if magic != PROTO_MAGIC or total_chunks == 0 or chunk_idx >= total_chunks:
                        bad_packets += 1
                    elif len(data) < HEADER_SIZE + payload_len:
                        bad_packets += 1
                    else:
                        payload = data[HEADER_SIZE : HEADER_SIZE + payload_len]
                        entry = pending_frames.get(frame_id)
                        if entry is None:
                            entry = {"total": int(total_chunks), "chunks": {}, "ts": now}
                            pending_frames[frame_id] = entry
                        elif entry["total"] != int(total_chunks):
                            pending_frames.pop(frame_id, None)
                            bad_packets += 1
                            entry = None

                        if entry is not None:
                            entry["ts"] = now
                            if chunk_idx not in entry["chunks"]:
                                entry["chunks"][int(chunk_idx)] = payload

                            if len(entry["chunks"]) == entry["total"]:
                                decoded = decode_complete_frame(entry)
                                pending_frames.pop(frame_id, None)
                                if decoded is not None:
                                    latest_frame = decoded
                                    latest_frame_id = frame_id
                                    decoded_frames += 1

            if pending_frames:
                stale_ids = []
                for frame_id, entry in pending_frames.items():
                    too_old = (now - entry["ts"]) > stale_timeout
                    too_far_behind = latest_frame_id >= 0 and frame_id < (latest_frame_id - 3)
                    if too_old or too_far_behind:
                        stale_ids.append(frame_id)
                for frame_id in stale_ids:
                    pending_frames.pop(frame_id, None)
                    dropped_frames += 1

            if (now - last_stats_ts) >= STATS_INTERVAL_S:
                dt = now - last_stats_ts
                rx_fps = (decoded_frames - decoded_frames_prev) / max(dt, 1e-6)
                decoded_frames_prev = decoded_frames
                last_stats_ts = now

                if rx_fps < (args.target_fps - 1.0):
                    display_scale = max(MIN_DISPLAY_SCALE, display_scale * 0.85)
                elif rx_fps > (args.target_fps + 4.0):
                    display_scale = min(MAX_DISPLAY_SCALE, display_scale * 1.05)

            if latest_frame is None:
                frame_canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
                cv2.putText(
                    frame_canvas,
                    "Waiting for UDP frames...",
                    (20, target_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (200, 200, 200),
                    2,
                    cv2.LINE_AA,
                )
            else:
                frame = latest_frame
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

            link_age_ms = int((now - last_rx_ts) * 1000.0) if last_rx_ts > 0 else -1
            status = (
                f"udp {args.host}:{args.port} | rx {rx_fps:.1f} fps | scale {display_scale:.2f} | "
                f"drop {dropped_frames} bad {bad_packets} | age {link_age_ms}ms | q quit"
            )
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
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
