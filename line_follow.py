#!/usr/bin/env python3
"""
Line follower for SparkyBot Mini (Mecanum drive) using front camera and PID steering.

Assumptions from user:
- FL = m1, BL = m2, FR = m3, BR = m4
- Black electrical tape on floor
- Straight speed = 50%
- Turn left by increasing RIGHT wheels (m3/m4)
- Turn right by increasing LEFT wheels (m1/m2)

This script keeps motor I/O in one place (MotorDriver class). If your
`sparkybotmini.py` API differs, only edit that class.
"""

import time
import inspect
import cv2
import numpy as np

# Import your robot library
import sparkybotmini


# ---------------------------
# Tunable parameters
# ---------------------------
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# Vision
ROI_Y_START_RATIO = 0.60   # bottom 40% of image
ROI_Y_END_RATIO = 0.95
BLUR_KERNEL = (5, 5)
THRESH_BINARY_INV = 70     # threshold for black tape (lower = darker)
MIN_CONTOUR_AREA = 500

# Control
BASE_SPEED_PERCENT = 50.0
MAX_SPEED_PERCENT = 90.0
MIN_SPEED_PERCENT = 0.0

# PID gains (start here, tune on robot)
KP = 0.35
KI = 0.00
KD = 0.18

INTEGRAL_LIMIT = 2000.0
LOST_LINE_TIMEOUT_S = 0.6
LOOP_DELAY_S = 0.01


class MotorDriver:
    """
    Adapter for sparkybotmini motor control.

    If your library uses different names/signatures, only change methods here.
    """

    def __init__(self):
        # Common pattern: instantiate robot/board object
        # Adjust if your library differs.
        self.bot = sparkybotmini.SparkyBotMini()

    def set_wheels_percent(self, m1, m2, m3, m4):
        """
        Set wheel speeds in percent [0..100] for forward movement.
        FL=m1, BL=m2, FR=m3, BR=m4.
        """
        m1 = float(np.clip(m1, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))
        m2 = float(np.clip(m2, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))
        m3 = float(np.clip(m3, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))
        m4 = float(np.clip(m4, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT))

        # Try likely SparkyBot API variants in order.
        # 1) set_motor(m1, m2, m3, m4)
        try:
            self.bot.set_motor(m1, m2, m3, m4)
            return
        except TypeError:
            pass

        # 2) set_motors(m1, m2, m3, m4)
        try:
            self.bot.set_motors(m1, m2, m3, m4)
            return
        except (TypeError, AttributeError):
            pass

        # 3) set_motor("m1", m1, m2, m3, m4) (channel/mode first)
        try:
            self.bot.set_motor("m1", m1, m2, m3, m4)
            return
        except TypeError:
            pass

        # Final diagnostic: report discovered signature for quick correction.
        sig = "unknown"
        try:
            sig = str(inspect.signature(self.bot.set_motor))
        except Exception:
            pass
        raise RuntimeError(
            f"Unsupported motor API for SparkyBotMini. set_motor signature: {sig}"
        )

    def stop(self):
        self.set_wheels_percent(0, 0, 0, 0)


class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error):
        now = time.time()
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now - self.prev_time

        p = self.kp * error

        if dt > 0.0:
            self.integral += error * dt
            if self.integral_limit is not None:
                self.integral = float(
                    np.clip(self.integral, -self.integral_limit, self.integral_limit)
                )
        i = self.ki * self.integral

        if dt > 0.0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d = self.kd * derivative

        self.prev_error = error
        self.prev_time = now

        return p + i + d


def find_line_error(frame):
    """
    Returns normalized error in [-1, 1], debug frame, and BW mask.
    error < 0 => line is left of center
    error > 0 => line is right of center
    """
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

    # Build full-frame BW view for display (ROI area contains thresholded mask)
    bw_full = np.zeros((h, w), dtype=np.uint8)
    bw_full[y0:y1, :] = mask

    if not contours:
        return None, debug, bw_full

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_CONTOUR_AREA:
        return None, debug, bw_full

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, debug, bw_full

    cx_roi = int(M["m10"] / M["m00"])
    cy_roi = int(M["m01"] / M["m00"])

    cx = cx_roi
    cy = cy_roi + y0

    image_center_x = w // 2

    error_pixels = cx - image_center_x
    error_norm = error_pixels / (w / 2.0)
    error_norm = float(np.clip(error_norm, -1.0, 1.0))

    cv2.drawContours(debug[y0:y1, :], [largest], -1, (0, 255, 0), 2)
    cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
    cv2.line(debug, (image_center_x, y0), (image_center_x, y1), (255, 255, 0), 2)
    cv2.putText(
        debug,
        f"err={error_norm:+.3f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return error_norm, debug, bw_full


def apply_steering(driver, base_speed, steer):
    """
    steer > 0 means line is right -> robot should turn right
    turn right by increasing LEFT wheels (m1,m2)

    steer < 0 means line is left -> robot should turn left
    turn left by increasing RIGHT wheels (m3,m4)
    """
    delta = abs(steer) * 50.0  # convert PID output scale to percent boost

    if steer > 0:
        # Turn right: boost left side
        left = base_speed + delta
        right = base_speed - delta * 0.5
    elif steer < 0:
        # Turn left: boost right side
        left = base_speed - delta * 0.5
        right = base_speed + delta
    else:
        left = right = base_speed

    m1 = left   # FL
    m2 = left   # BL
    m3 = right  # FR
    m4 = right  # BR

    driver.set_wheels_percent(m1, m2, m3, m4)


def main():
    driver = MotorDriver()
    pid = PID(KP, KI, KD, integral_limit=INTEGRAL_LIMIT)

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
                    driver.stop()
                cv2.putText(
                    debug,
                    "LINE LOST",
                    (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
            else:
                last_seen_time = time.time()
                steer = pid.update(error)
                steer = float(np.clip(steer, -1.0, 1.0))
                apply_steering(driver, BASE_SPEED_PERCENT, steer)

                cv2.putText(
                    debug,
                    f"steer={steer:+.3f}",
                    (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            # Show debug overlay only
            cv2.imshow("Line Follower Debug", debug)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(LOOP_DELAY_S)

    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()
        cap.release()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == "__main__":
    main()
