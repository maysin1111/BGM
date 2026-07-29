#!/usr/bin/env python3
"""
Line follower for SparkyBot Mini (Mecanum drive) using front camera and PID steering.
Patched with startup motor diagnostics + wheel invert/remap.
"""

import time
import cv2
import numpy as np
import sparkybotmini

# ---------------------------
# Tunable parameters
# ---------------------------
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# Vision
ROI_Y_START_RATIO = 0.60
ROI_Y_END_RATIO = 0.95
BLUR_KERNEL = (5, 5)
THRESH_BINARY_INV = 70
MIN_CONTOUR_AREA = 500

# Control
BASE_SPEED_PERCENT = 45.0
MAX_SPEED_PERCENT = 90.0
MIN_SPEED_PERCENT = -90.0
STEER_DEADBAND = 0.05

# PID
KP = 0.20
KI = 0.00
KD = 0.25

INTEGRAL_LIMIT = 2000.0
LOST_LINE_TIMEOUT_S = 0.6
LOOP_DELAY_S = 0.01

# ---------------------------
# Motor mapping / polarity patch
# ---------------------------
# Command order expected by robot.set_motor is (m1, m2, m3, m4).
# If robot doesn't move as expected, change MOTOR_MAP and/or INVERT_M*.
MOTOR_MAP = (0, 1, 2, 3)  # identity map
INVERT_M1 = False
INVERT_M2 = False
INVERT_M3 = False
INVERT_M4 = False

# Startup diagnostics
ENABLE_STARTUP_MOTOR_TEST = True
STARTUP_TEST_SPEED = 35
STARTUP_TEST_DURATION = 0.7


class MotorDriver:
    def __init__(self):
        self.bot = sparkybotmini.SparkyBotMini(port="/dev/ttyAMA10", debug=False)
        self.comm_failed = False

        if not self.bot.connect():
            self.comm_failed = True
            raise RuntimeError(
                f"Failed to connect SparkyBotMini on port {self.bot.port}. "
                "Check USB/port/power."
            )

        # Enable reports so we can read encoders/voltage for debugging.
        self.bot.set_auto_report(True)
        time.sleep(0.1)

        ver = self.bot.get_version(timeout=0.1)
        batt = self.bot.get_battery_voltage()
        print(f"[INIT] Connected on {self.bot.port} | fw={ver} | batt={batt:.2f}V")

    def _is_connected(self):
        return (self.bot.ser is not None) and bool(getattr(self.bot.ser, "is_open", False))

    @staticmethod
    def _clamp(v):
        return int(np.clip(int(round(v)), -100, 100))

    def _apply_map_and_invert(self, m1, m2, m3, m4):
        vals = [m1, m2, m3, m4]

        # map logical wheels -> physical channels
        vals = [vals[MOTOR_MAP[0]], vals[MOTOR_MAP[1]], vals[MOTOR_MAP[2]], vals[MOTOR_MAP[3]]]

        # per-channel inversion
        if INVERT_M1:
            vals[0] = -vals[0]
        if INVERT_M2:
            vals[1] = -vals[1]
        if INVERT_M3:
            vals[2] = -vals[2]
        if INVERT_M4:
            vals[3] = -vals[3]

        return vals

    def set_wheels_percent(self, m1, m2, m3, m4):
        m1 = float(np.clip(m1, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))
        m2 = float(np.clip(m2, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))
        m3 = float(np.clip(m3, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))
        m4 = float(np.clip(m4, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))

        if self.comm_failed:
            raise RuntimeError("Motor communication is in failed state.")
        if not self._is_connected():
            self.comm_failed = True
            raise RuntimeError("Serial not connected (ser is None or closed).")

        c1, c2, c3, c4 = self._apply_map_and_invert(
            self._clamp(m1), self._clamp(m2), self._clamp(m3), self._clamp(m4)
        )

        print(f"[MOTOR] cmd raw=({m1:+.1f},{m2:+.1f},{m3:+.1f},{m4:+.1f}) -> sent=({c1:+d},{c2:+d},{c3:+d},{c4:+d})")

        try:
            self.bot.set_motor(c1, c2, c3, c4)
        except Exception as e:
            self.comm_failed = True
            raise RuntimeError(f"Motor send failed: {e}") from e

    def stop(self):
        if self.comm_failed:
            return
        try:
            self.set_wheels_percent(0, 0, 0, 0)
        except Exception:
            self.comm_failed = True

    def startup_test(self):
        """
        Quick test to confirm motors physically respond.
        """
        print("[TEST] Running startup motor test...")

        sequences = [
            ("forward",  STARTUP_TEST_SPEED,  STARTUP_TEST_SPEED,  STARTUP_TEST_SPEED,  STARTUP_TEST_SPEED),
            ("backward", -STARTUP_TEST_SPEED, -STARTUP_TEST_SPEED, -STARTUP_TEST_SPEED, -STARTUP_TEST_SPEED),
            ("turn_right", STARTUP_TEST_SPEED, STARTUP_TEST_SPEED, -STARTUP_TEST_SPEED, -STARTUP_TEST_SPEED),
            ("turn_left", -STARTUP_TEST_SPEED, -STARTUP_TEST_SPEED, STARTUP_TEST_SPEED, STARTUP_TEST_SPEED),
        ]

        for name, a, b, c, d in sequences:
            print(f"[TEST] {name}")
            self.set_wheels_percent(a, b, c, d)
            time.sleep(STARTUP_TEST_DURATION)
            self.set_wheels_percent(0, 0, 0, 0)
            time.sleep(0.25)

            try:
                enc = self.bot.get_encoders()
                print(f"[TEST] encoders after {name}: {enc}")
            except Exception as e:
                print(f"[TEST] encoder read failed: {e}")

        print("[TEST] Startup motor test done.")

    def close(self):
        try:
            self.bot.disconnect()
        except Exception:
            pass


class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error):
        now = time.time()
        dt = 0.0 if self.prev_time is None else (now - self.prev_time)

        p = self.kp * error

        if dt > 0.0:
            self.integral += error * dt
            if self.integral_limit is not None:
                self.integral = float(np.clip(self.integral, -self.integral_limit, self.integral_limit))
        i = self.ki * self.integral

        derivative = 0.0 if dt == 0.0 else (error - self.prev_error) / dt
        d = self.kd * derivative

        self.prev_error = error
        self.prev_time = now
        return p + i + d


def find_line_error(frame):
    h, w = frame.shape[:2]

    y0 = int(h * ROI_Y_START_RATIO)
    y1 = int(h * ROI_Y_END_RATIO)
    roi = frame[y0:y1, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)

    _, mask = cv2.threshold(blur, THRESH_BINARY_INV, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    debug = frame.copy()
    cv2.rectangle(debug, (0, y0), (w, y1), (255, 0, 0), 2)

    bw_full = np.zeros((h, w), dtype=np.uint8)
    bw_full[y0:y1, :] = mask

    if not contours:
        cv2.putText(debug, "no contours", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 120, 255), 2, cv2.LINE_AA)
        return None, debug, bw_full

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    cv2.putText(debug, f"area={area:.0f}", (10, 125),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    if area < MIN_CONTOUR_AREA:
        cv2.putText(debug, "contour too small", (10, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 120, 255), 2, cv2.LINE_AA)
        return None, debug, bw_full

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, debug, bw_full

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"]) + y0
    image_center_x = w // 2

    error_norm = float(np.clip((cx - image_center_x) / (w / 2.0), -1.0, 1.0))

    cv2.drawContours(debug[y0:y1, :], [largest], -1, (0, 255, 0), 2)
    cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
    cv2.line(debug, (image_center_x, y0), (image_center_x, y1), (255, 255, 0), 2)
    cv2.putText(
        debug, f"err={error_norm:+.3f}", (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA
    )

    return error_norm, debug, bw_full


def apply_steering(driver, base_speed, steer):
    if abs(steer) < STEER_DEADBAND:
        steer = 0.0

    delta = abs(steer) * 35.0

    if steer > 0:
        left = base_speed + delta
        right = base_speed - delta * 0.5
    elif steer < 0:
        left = base_speed - delta * 0.5
        right = base_speed + delta
    else:
        left = right = base_speed

    # logical order: FL, BL, FR, BR
    driver.set_wheels_percent(left, left, right, right)


def main():
    driver = MotorDriver()
    pid = PID(KP, KI, KD, integral_limit=INTEGRAL_LIMIT)

    if ENABLE_STARTUP_MOTOR_TEST:
        driver.startup_test()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    last_seen_time = time.time()
    print("Starting line follower. Press 'q' in display window or Ctrl+C to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                driver.stop()
                time.sleep(0.05)
                continue

            error, debug, _ = find_line_error(frame)

            if error is None:
                if time.time() - last_seen_time > LOST_LINE_TIMEOUT_S:
                    try:
                        driver.stop()
                    except Exception as e:
                        print(f"Motor stop failed: {e}")
                        driver.comm_failed = False

                cv2.putText(
                    debug, "LINE LOST", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA
                )
            else:
                last_seen_time = time.time()
                steer = float(np.clip(pid.update(error), -1.0, 1.0))

                try:
                    apply_steering(driver, BASE_SPEED_PERCENT, steer)
                except Exception as e:
                    print(f"Motor communication failed: {e}")
                    print("Will retry next loop...")
                    driver.comm_failed = False

                cv2.putText(
                    debug, f"steer={steer:+.3f}", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA
                )

            cv2.imshow("Line Follower Debug", debug)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(LOOP_DELAY_S)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            driver.stop()
        except Exception:
            pass
        driver.close()
        cap.release()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == "__main__":
    main()
