#!/usr/bin/env python3
"""
Combined WASD Robot Controller + Webcam Vision

Runs both robot control and webcam green object detection side-by-side.
- Left: Robot WASD controls with keyboard input
- Right: Live camera feed with green object (corn) detection

Controls:
  w = forward,  s = back,  a = strafe left,  d = strafe right
  q = rotate left,  e = rotate right
  x = quit

Requirements:
  - sparkybotmini.py must be on PYTHONPATH
  - pip install pyserial opencv-python numpy

Run: python3 wasdContol_webcam_v4.py
"""

import time
import sys
import argparse
import threading
import cv2
import numpy as np

# Import the robot API
try:
    from sparkybotmini import SparkyBotMini
except Exception as e:
    print("Failed to import SparkyBotMini from sparkybotmini.py.")
    print("Import error:", e)
    sys.exit(1)


# Global state
keys_pressed = set()
should_exit = False
camera_running = True
frame_lock = threading.Lock()
current_frame = None
current_mask = None


def input_thread_func():
    """Background thread that reads keyboard input character by character."""
    global keys_pressed, should_exit
    
    print("[DEBUG] Input thread started. Type keys directly (no Enter needed)...", file=sys.stderr)
    
    while not should_exit:
        try:
            ch = sys.stdin.read(1)
            
            if not ch:
                continue
            
            ch_lower = ch.lower()
            
            if ch_lower == 'x':
                print("\n[DEBUG] Exit key pressed", file=sys.stderr)
                should_exit = True
                break
            
            if ch_lower in 'wsadqe':
                if ch_lower not in keys_pressed:
                    keys_pressed.add(ch_lower)
                    print(f"[KEY] Pressed: {ch_lower}", file=sys.stderr)
                    
                    time.sleep(0.1)
                    keys_pressed.discard(ch_lower)
                    print(f"[KEY] Released: {ch_lower}", file=sys.stderr)
            
        except EOFError:
            should_exit = True
            break
        except Exception as e:
            print(f"[ERROR] Input thread error: {e}", file=sys.stderr)
            time.sleep(0.1)


def camera_thread_func():
    """Background thread that captures and processes video."""
    global camera_running, current_frame, current_mask
    
    print("[DEBUG] Camera thread started...", file=sys.stderr)
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("[ERROR] Could not open camera.")
        camera_running = False
        return
    
    print("[DEBUG] Camera opened successfully!", file=sys.stderr)
    
    while camera_running and not should_exit:
        ret, image = cap.read()
        
        if not ret:
            print("[ERROR] Failed to grab frame.")
            break
        
        # Convert to HSV for green detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Green color range
        lower_green = np.array([30, 40, 40])
        upper_green = np.array([100, 255, 255])
        
        # Create mask
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > 500:
                x, y, w, h = cv2.boundingRect(largest)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Draw contour
                cv2.drawContours(image, [largest], -1, (0, 0, 255), 2)
                
                # Draw bounding box
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Label
                cv2.putText(image, "Corn", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # Center dot
                cv2.circle(image, (center_x, center_y), 6, (255, 0, 0), -1)
                
                # Crosshair
                cv2.line(image, (center_x - 10, center_y), (center_x + 10, center_y), (255, 0, 0), 2)
                cv2.line(image, (center_x, center_y - 10), (center_x, center_y + 10), (255, 0, 0), 2)
                
                # Coordinates
                cv2.putText(image, f"({center_x}, {center_y})", (center_x + 15, center_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Store frames with lock
        with frame_lock:
            current_frame = image.copy()
            current_mask = mask.copy()
        
        time.sleep(0.03)
    
    cap.release()
    cv2.destroyAllWindows()
    camera_running = False


def is_pressed(key):
    """Check if a key is currently pressed."""
    return key.lower() in keys_pressed


def clamp_int(v: float, lo: int = -100, hi: int = 100) -> int:
    return max(lo, min(hi, int(round(v))))


def stop_all(robot: SparkyBotMini):
    try:
        robot.set_motor(0, 0, 0, 0)
    except Exception as e:
        print("Warning: failed to stop motors:", e)


def main():
    global should_exit, camera_running
    
    parser = argparse.ArgumentParser(description="WASD Controller + Webcam Vision")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--max", type=int, default=80, help="Maximum wheel speed (0-100). Default 80")
    parser.add_argument("--debug", "-d", action="store_true", help="Enable debug output")
    args = parser.parse_args()

    MAX_SPEED = max(0, min(100, args.max))

    print(f"[DEBUG] Initializing SparkyBotMini with port={args.port}, baud={args.baud}")
    robot = SparkyBotMini(port=args.port, baudrate=args.baud, debug=args.debug)

    print(f"[DEBUG] Connecting to SparkyBotMini on {args.port} @ {args.baud}...")
    if not robot.connect():
        print("Failed to open serial port. Exiting.")
        sys.exit(1)

    print("[DEBUG] Connection successful!")
    
    # Initialize robot
    print("[DEBUG] Stopping all motors...")
    stop_all(robot)
    time.sleep(0.05)

    print("[DEBUG] Enabling auto-report...")
    robot.set_auto_report(True)
    time.sleep(0.2)
    
    # Test motors
    print("[DEBUG] Testing motors...")
    robot.set_motor(50, 50, 50, 50)
    time.sleep(1)
    robot.set_motor(0, 0, 0, 0)
    time.sleep(0.2)

    print("\n=== WASD Controller + Webcam Ready ===")
    print("Controls: w=forward, s=back, a=left, d=right, q/e=rotate, x=quit")
    print(f"Max speed: {MAX_SPEED}\n")
    
    # Start input thread
    try:
        input_thread = threading.Thread(target=input_thread_func, daemon=True)
        input_thread.start()
        print("[DEBUG] Input thread started!", file=sys.stderr)
        
        # Start camera thread
        camera_thread = threading.Thread(target=camera_thread_func, daemon=True)
        camera_thread.start()
        print("[DEBUG] Camera thread started!", file=sys.stderr)
        
        time.sleep(0.5)
    except Exception as e:
        print(f"[ERROR] Failed to start threads: {e}")
        robot.disconnect()
        sys.exit(1)
    
    last_m1, last_m2, last_m3, last_m4 = 0, 0, 0, 0
    command_count = 0
    
    try:
        while not should_exit and camera_running:
            # components
            f = 0.0    # forward
            s = 0.0    # strafe
            rot = 0.0  # rotation

            if is_pressed("w"):
                f += MAX_SPEED
            if is_pressed("s"):
                f -= MAX_SPEED
            if is_pressed("d"):
                s += MAX_SPEED
            if is_pressed("a"):
                s -= MAX_SPEED
            if is_pressed("e"):
                rot += MAX_SPEED * 0.6
            if is_pressed("q"):
                rot -= MAX_SPEED * 0.6

            # Mecanum X-configuration mixing
            m1 = f + s + rot
            m2 = f - s + rot
            m3 = f - s - rot
            m4 = f + s - rot

            # Normalize if necessary
            max_cmd = max(abs(m1), abs(m2), abs(m3), abs(m4), MAX_SPEED)
            if max_cmd > MAX_SPEED:
                scale = MAX_SPEED / max_cmd
                m1 *= scale; m2 *= scale; m3 *= scale; m4 *= scale

            # Clamp to valid range
            m1_clamped = clamp_int(m1)
            m2_clamped = clamp_int(m2)
            m3_clamped = clamp_int(m3)
            m4_clamped = clamp_int(m4)
            
            try:
                robot.set_motor(m1_clamped, m2_clamped, m3_clamped, m4_clamped)
                command_count += 1
                
                if (m1_clamped, m2_clamped, m3_clamped, m4_clamped) != (last_m1, last_m2, last_m3, last_m4):
                    print(f"[{command_count}] Motors: M1={m1_clamped:4d}, M2={m2_clamped:4d}, M3={m3_clamped:4d}, M4={m4_clamped:4d}")
                    last_m1, last_m2, last_m3, last_m4 = m1_clamped, m2_clamped, m3_clamped, m4_clamped
                    
            except Exception as e:
                print(f"[ERROR] Motor command error: {e}")
                stop_all(robot)
                break

            # Display camera feed if available
            with frame_lock:
                if current_frame is not None:
                    cv2.imshow("Camera - Green Corn Detector", current_frame)
                if current_mask is not None:
                    cv2.imshow("Mask", current_mask)
            
            # Check for window close or 'q' key
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or not cv2.getWindowProperty("Camera - Green Corn Detector", cv2.WND_PROP_VISIBLE):
                should_exit = True
                break

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[DEBUG] Interrupted by user")

    finally:
        print("[DEBUG] Shutting down...")
        should_exit = True
        camera_running = False
        stop_all(robot)
        time.sleep(0.1)
        robot.disconnect()
        cv2.destroyAllWindows()
        print("[DEBUG] Exited cleanly.")


if __name__ == "__main__":
    main()
