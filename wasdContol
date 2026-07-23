#!/usr/bin/env python3
"""
WASD controller for a 4-motor mecanum robot (X formation) using sparkybotmini.SparkyBotMini.

Motor mapping:
  motor 1 = front left
  motor 2 = back  left
  motor 3 = front right
  motor 4 = back  right

Controls:
  W = forward
  S = backward
  A = strafe left
  D = strafe right
  Q = rotate left (optional)
  E = rotate right (optional)
  ESC or Ctrl-C = stop and exit

Requirements:
  - sparkybotmini.py must be on PYTHONPATH (this repo contains it).
  - pip install keyboard pyserial
  - On Linux, keyboard may require sudo.
"""

import time
import sys
import argparse

try:
    import keyboard
except Exception as e:
    print("Missing dependency: the 'keyboard' module is required. Install with: pip install keyboard")
    print("Note: on Linux 'keyboard' usually needs sudo privileges to capture global keys.")
    print("Error:", e)
    sys.exit(1)

# Import the actual API from repo
try:
    from sparkybotmini import SparkyBotMini
except Exception as e:
    print("Failed to import SparkyBotMini from sparkybotmini.py. Ensure sparkybotmini.py is on PYTHONPATH.")
    print("Import error:", e)
    sys.exit(1)


def clamp_int(v: float, lo: int = -100, hi: int = 100) -> int:
    return max(lo, min(hi, int(round(v))))


def stop_all(robot: SparkyBotMini):
    try:
        robot.set_motor(0, 0, 0, 0)
    except Exception as e:
        print("Warning: failed to stop motors:", e)


def main():
    parser = argparse.ArgumentParser(description="WASD controller for SparkyBotMini mecanum (X formation).")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--max", type=int, default=80, help="Maximum wheel speed (0-100). Default 80")
    args = parser.parse_args()

    MAX_SPEED = max(0, min(100, args.max))

    robot = SparkyBotMini(port=args.port, baudrate=args.baud, debug=False)

    print("Connecting to SparkyBotMini on", args.port, "@", args.baud)
    if not robot.connect():
        print("Failed to open serial port. Exiting.")
        sys.exit(1)

    # Ensure motors start at 0
    stop_all(robot)
    time.sleep(0.05)

    print("Controls: W forward, S back, A strafe left, D strafe right, Q/E rotate, ESC to quit")
    try:
        while True:
            # components
            f = 0.0    # forward (+ forward)
            s = 0.0    # strafe (+ right)
            rot = 0.0  # rotation (+ clockwise)

            if keyboard.is_pressed("w"):
                f += MAX_SPEED
            if keyboard.is_pressed("s"):
                f -= MAX_SPEED
            if keyboard.is_pressed("d"):
                s += MAX_SPEED
            if keyboard.is_pressed("a"):
                s -= MAX_SPEED
            if keyboard.is_pressed("e"):
                rot += MAX_SPEED * 0.6  # reduced rotation scale
            if keyboard.is_pressed("q"):
                rot -= MAX_SPEED * 0.6

            # Exit on ESC
            if keyboard.is_pressed("esc"):
                print("ESC pressed, exiting loop.")
                break

            # Mecanum X-configuration mixing (uses SparkyBotMini motor ordering)
            # motor1 = front left, motor2 = back left, motor3 = front right, motor4 = back right
            m1 = f + s + rot
            m2 = f - s + rot
            m3 = f - s - rot
            m4 = f + s - rot

            # Normalize to allowed range if necessary
            max_cmd = max(abs(m1), abs(m2), abs(m3), abs(m4), MAX_SPEED)
            if max_cmd > MAX_SPEED:
                scale = MAX_SPEED / max_cmd
                m1 *= scale; m2 *= scale; m3 *= scale; m4 *= scale

            # Convert to ints and clamp to -100..100 which SparkyBotMini.set_motor expects
            try:
                robot.set_motor(clamp_int(m1), clamp_int(m2), clamp_int(m3), clamp_int(m4))
            except Exception as e:
                print("Error sending motor command:", e)
                # try to stop motors and exit
                stop_all(robot)
                break

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Stopping motors and disconnecting...")
        stop_all(robot)
        time.sleep(0.05)
        robot.disconnect()
        print("Exited cleanly.")


if __name__ == "__main__":
    main()
