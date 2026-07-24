#!/usr/bin/env python3
"""
WASD controller for a 4-motor mecanum robot (X formation) using sparkybotmini.SparkyBotMini.

run in terminal via: python3 /home/pi/bgm.py/wasdContol.py

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
  - pip install pynput pyserial
"""

import time
import sys
import argparse

try:
    from pynput import keyboard as kb
except Exception as e:
    print("Missing dependency: the 'pynput' module is required. Install with: pip install pynput")
    print("Error:", e)
    sys.exit(1)

# Import the actual API from repo
try:
    from sparkybotmini import SparkyBotMini
except Exception as e:
    print("Failed to import SparkyBotMini from sparkybotmini.py. Ensure sparkybotmini.py is on PYTHONPATH.")
    print("Import error:", e)
    sys.exit(1)


# Global state for key tracking
keys_pressed = set()


def on_press(key):
    try:
        keys_pressed.add(key.char.lower())
    except AttributeError:
        # Special keys (ESC, etc.)
        if key == kb.Key.esc:
            keys_pressed.add('esc')
        elif key == kb.Key.shift:
            keys_pressed.add('shift')


def on_release(key):
    try:
        keys_pressed.discard(key.char.lower())
    except AttributeError:
        if key == kb.Key.esc:
            keys_pressed.discard('esc')
        elif key == kb.Key.shift:
            keys_pressed.discard('shift')


def is_pressed(key):
    """Check if a key is currently pressed."""
    return key in keys_pressed


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
    parser.add_argument("--debug", "-d", action="store_true", help="Enable debug output")
    args = parser.parse_args()

    MAX_SPEED = max(0, min(100, args.max))

    print(f"[DEBUG] Initializing SparkyBotMini with port={args.port}, baud={args.baud}, debug={args.debug}")
    robot = SparkyBotMini(port=args.port, baudrate=args.baud, debug=args.debug)

    print(f"[DEBUG] Connecting to SparkyBotMini on {args.port} @ {args.baud}...")
    if not robot.connect():
        print("Failed to open serial port. Exiting.")
        sys.exit(1)

    print("[DEBUG] Connection successful!")
    
    # Ensure motors start at 0
    print("[DEBUG] Stopping all motors...")
    stop_all(robot)
    time.sleep(0.05)

    # Enable auto-reporting (required for motor control)
    print("[DEBUG] Enabling auto-report...")
    robot.set_auto_report(True)
    time.sleep(0.2)
    
    # Test motors with a simple command
    print("[DEBUG] Testing motors with a simple 50% forward command...")
    robot.set_motor(50, 50, 50, 50)
    time.sleep(1)
    print("[DEBUG] Test command sent. Robot should have moved. Stopping...")
    robot.set_motor(0, 0, 0, 0)
    time.sleep(0.2)

    print("\n=== WASD Controller Ready ===")
    print("Controls: W forward, S back, A strafe left, D strafe right, Q/E rotate, ESC to quit")
    print(f"Max speed: {MAX_SPEED}")
    print("Press keys now...\n")
    
    # Start keyboard listener
    listener = kb.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    last_m1, last_m2, last_m3, last_m4 = 0, 0, 0, 0
    command_count = 0
    
    try:
        while True:
            # components
            f = 0.0    # forward (+ forward)
            s = 0.0    # strafe (+ right)
            rot = 0.0  # rotation (+ clockwise)

            if is_pressed("w"):
                f += MAX_SPEED
            if is_pressed("s"):
                f -= MAX_SPEED
            if is_pressed("d"):
                s += MAX_SPEED
            if is_pressed("a"):
                s -= MAX_SPEED
            if is_pressed("e"):
                rot += MAX_SPEED * 0.6  # reduced rotation scale
            if is_pressed("q"):
                rot -= MAX_SPEED * 0.6

            # Exit on ESC
            if is_pressed("esc"):
                print("\n[DEBUG] ESC pressed, exiting loop.")
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
            m1_clamped = clamp_int(m1)
            m2_clamped = clamp_int(m2)
            m3_clamped = clamp_int(m3)
            m4_clamped = clamp_int(m4)
            
            try:
                robot.set_motor(m1_clamped, m2_clamped, m3_clamped, m4_clamped)
                command_count += 1
                
                # Debug output: print when values change
                if (m1_clamped, m2_clamped, m3_clamped, m4_clamped) != (last_m1, last_m2, last_m3, last_m4):
                    print(f"[{command_count}] Motors: M1={m1_clamped:4d}, M2={m2_clamped:4d}, M3={m3_clamped:4d}, M4={m4_clamped:4d} | f={f:.0f} s={s:.0f} rot={rot:.0f}")
                    last_m1, last_m2, last_m3, last_m4 = m1_clamped, m2_clamped, m3_clamped, m4_clamped
                    
            except Exception as e:
                print(f"[ERROR] Error sending motor command: {e}")
                # try to stop motors and exit
                stop_all(robot)
                break

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[DEBUG] Interrupted by user")

    finally:
        print("[DEBUG] Stopping motors and disconnecting...")
        stop_all(robot)
        time.sleep(0.05)
        robot.disconnect()
        listener.stop()
        print("[DEBUG] Exited cleanly.")


if __name__ == "__main__":
    main()
