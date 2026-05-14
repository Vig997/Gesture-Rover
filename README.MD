# ECE5_GestureRover

Computer-vision steering prototype for a differential-drive rover.

![Wavelink Rover Poster](./Gesture%20Rover.png)

## What It Does

- Tracks two hands as a steering wheel using MediaPipe.
- Computes:
  - steering angle (`deg`)
  - width scalar (`w`)
  - reversing flag
- Converts CV outputs into wheel-control packets (`plan.md` format).
- Shows live packet preview and interpreted car motion in the tracker overlay.

## Key Files

- `comp_vision_input/steering_wheel_tracker.py`
  - Main live tracker + packet preview HUD.
- `comp_vision_input/cv_to_packet.py`
  - Width/angle -> left/right wheel mapping and packet encoding.
- `comp_vision_input/gesture_to_esp.py`
  - TCP bridge framework with customizable payload hooks.
- `esp32/wifi_connect.ino`
  - ESP32 Wi-Fi TCP receiver sketch.
- `plan.md`
  - Packet field spec.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run

Tracker with live packet preview:

```bash
python comp_vision_input/steering_wheel_tracker.py
```

Direct packet conversion test:

```bash
python comp_vision_input/cv_to_packet.py --width 1.11 --angle -40.3
```

ESP bridge framework:

```bash
python comp_vision_input/gesture_to_esp.py --host <ESP_IP> --port 12345
```

## Packet Format (`plan.md`)

8-character ASCII string:

`<ReverseLeft><SpeedLeft:3><ReverseRight><SpeedRight:3>`

Example:

`11800110`

- Reverse Left = `1`
- Speed Left = `180`
- Reverse Right = `0`
- Speed Right = `110`

## Current Control Rules (`cv_to_packet.py`)

- Speed cap: `w > 1.7` -> treated as `1.7`
- Brake floor: `w < 0.2` -> treated as `0`
- Differential-drive turning:
  - left turn (negative angle) -> right wheel faster
  - right turn (positive angle) -> left wheel faster
- Turning can happen at low/zero forward speed (pivot behavior).
- If one wheel would exceed max PWM, both are scaled together to preserve turn ratio.

## Width Scaling

Tracker width (`w`) is screen-based:

- `w` is derived from hand distance in pixels vs frame width.
- Near full-frame hand span maps near `2.00x`.

## ESP32 Note

`esp32/wifi_connect.ino` currently expects fixed-size binary frames (`<Bhh>`), while `cv_to_packet.py` outputs ASCII control packets from `plan.md`.

Before full integration, align both sides to one protocol.
