import argparse
from dataclasses import dataclass


MIN_WIDTH_BRAKE = 0.2
MAX_WIDTH_SPEED = 1.7
MAX_PWM = 255
MAX_STEER_DEG = 180.0
PIVOT_TURN_FLOOR = 0.35
TURN_DEADBAND_DEG = 8.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_deg_180(angle_deg: float) -> float:
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def normalize_speed(width_x: float) -> float:
    limited = clamp(width_x, 0.0, MAX_WIDTH_SPEED)
    if limited < MIN_WIDTH_BRAKE:
        return 0.0
    return (limited - MIN_WIDTH_BRAKE) / (MAX_WIDTH_SPEED - MIN_WIDTH_BRAKE)


def normalize_turn(angle_deg: float) -> float:
    if abs(angle_deg) < TURN_DEADBAND_DEG:
        return 0.0
    return clamp(angle_deg / MAX_STEER_DEG, -1.0, 1.0)


@dataclass
class WheelState:
    reverse: int
    speed: int


def signed_to_wheel(value: float) -> WheelState:
    clipped = clamp(value, -1.0, 1.0)
    reverse = 1 if clipped < 0.0 else 0
    speed = int(round(abs(clipped) * MAX_PWM))
    return WheelState(reverse=reverse, speed=speed)


def width_angle_to_wheels(width_x: float, angle_deg: float, reversing: bool = False) -> tuple[WheelState, WheelState]:
    speed_norm = normalize_speed(width_x)
    drive_angle_deg = wrap_deg_180(angle_deg + 180.0) if reversing else angle_deg
    turn_norm = normalize_turn(drive_angle_deg)

    turn_scale = PIVOT_TURN_FLOOR + (1.0 - PIVOT_TURN_FLOOR) * speed_norm
    turn_component = turn_norm * turn_scale

    # Steering convention:
    # negative angle (left turn) -> right wheel faster
    # positive angle (right turn) -> left wheel faster
    left_signed = speed_norm + turn_component
    right_signed = speed_norm - turn_component

    if reversing:
        left_signed = -left_signed
        right_signed = -right_signed

    # Preserve left/right turn ratio when one side would otherwise clip.
    # Scale both channels together so max magnitude is exactly 1.0.
    max_mag = max(abs(left_signed), abs(right_signed), 1e-9)
    if max_mag > 1.0:
        left_signed /= max_mag
        right_signed /= max_mag

    return signed_to_wheel(left_signed), signed_to_wheel(right_signed)


def build_packet(left: WheelState, right: WheelState) -> str:
    return f"{left.reverse}{left.speed:03d}{right.reverse}{right.speed:03d}"


def width_angle_to_packet(width_x: float, angle_deg: float, reversing: bool = False) -> str:
    left, right = width_angle_to_wheels(width_x, angle_deg, reversing=reversing)
    return build_packet(left, right)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=float, required=True, help="CV width scalar (0..2-ish)")
    ap.add_argument("--angle", type=float, required=True, help="Steering angle in degrees (-180..180)")
    ap.add_argument("--reversing", action="store_true", help="Flip both wheel directions for reverse drive mode")
    args = ap.parse_args()

    packet = width_angle_to_packet(args.width, args.angle, reversing=args.reversing)
    print(packet)


if __name__ == "__main__":
    main()
