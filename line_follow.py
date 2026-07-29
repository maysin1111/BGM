#!/usr/bin/env python3
import time
import cv2
import numpy as np
import sparkybotmini

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# ---- debug controls ----
VERBOSE_MOTOR_STREAM = False      # no per-loop spam
RUN_LINE_FOLLOWER = False         # run only motor diagnostics first
ENABLE_STARTUP_MOTOR_TEST = True

TEST_SPEED = 40
TEST_TIME = 1.0

# Try alternates if no movement
PORT_CANDIDATES = ["/dev/ttyAMA10", "/dev/ttyUSB0", "/dev/ttyS0"]

# Mapping/invert tuning
MOTOR_MAP = (0, 1, 2, 3)  # m1,m2,m3,m4
INV = [False, False, False, False]


class MotorDriver:
    def __init__(self):
        self.bot = None
        self.port = None

        last_err = None
        for p in PORT_CANDIDATES:
            try:
                b = sparkybotmini.SparkyBotMini(port=p, debug=False)
                if b.connect():
                    self.bot = b
                    self.port = p
                    break
            except Exception as e:
                last_err = e

        if self.bot is None:
            raise RuntimeError(f"Could not connect on any port {PORT_CANDIDATES}. last_err={last_err}")

        self.bot.set_auto_report(True)
        time.sleep(0.2)
        print(f"[INIT] connected port={self.port}")
        try:
            print(f"[INIT] fw={self.bot.get_version(timeout=0.2)} batt={self.bot.get_battery_voltage():.2f}V")
        except Exception as e:
            print(f"[INIT] version/battery read failed: {e}")

    @staticmethod
    def _clamp(v):
        return int(np.clip(int(round(v)), -100, 100))

    def _map_inv(self, m1, m2, m3, m4):
        vals = [m1, m2, m3, m4]
        vals = [vals[MOTOR_MAP[0]], vals[MOTOR_MAP[1]], vals[MOTOR_MAP[2]], vals[MOTOR_MAP[3]]]
        for i in range(4):
            if INV[i]:
                vals[i] = -vals[i]
        return vals

    def set_motor(self, m1, m2, m3, m4, label=""):
        a, b, c, d = self._map_inv(self._clamp(m1), self._clamp(m2), self._clamp(m3), self._clamp(m4))
        print(f"[CMD] {label:<10} -> ({a:+d},{b:+d},{c:+d},{d:+d})")
        self.bot.set_motor(a, b, c, d)
        if VERBOSE_MOTOR_STREAM:
            print("[DBG] command sent")

    def stop(self):
        self.bot.set_motor(0, 0, 0, 0)

    def encoder_snapshot(self, tag):
        try:
            e = self.bot.get_encoders()
            print(f"[ENC] {tag}: {e}")
        except Exception as ex:
            print(f"[ENC] {tag}: read failed: {ex}")

    def startup_test(self):
        print("\n=== STARTUP MOTOR TEST ===")
        self.encoder_snapshot("before")

        steps = [
            ("forward",   +TEST_SPEED, +TEST_SPEED, +TEST_SPEED, +TEST_SPEED),
            ("backward",  -TEST_SPEED, -TEST_SPEED, -TEST_SPEED, -TEST_SPEED),
            ("right turn",+TEST_SPEED, +TEST_SPEED, -TEST_SPEED, -TEST_SPEED),
            ("left turn", -TEST_SPEED, -TEST_SPEED, +TEST_SPEED, +TEST_SPEED),
            ("strafe?",   +TEST_SPEED, -TEST_SPEED, -TEST_SPEED, +TEST_SPEED),  # mecanum pattern
        ]

        for name, m1, m2, m3, m4 in steps:
            input(f"\nPress ENTER for step: {name}")
            self.encoder_snapshot(f"{name} pre")
            self.set_motor(m1, m2, m3, m4, label=name)
            time.sleep(TEST_TIME)
            self.stop()
            time.sleep(0.3)
            self.encoder_snapshot(f"{name} post")

        print("\n=== TEST COMPLETE ===")
        print("If encoders changed but no movement -> motor power/mechanical issue.")
        print("If encoders never change -> wrong serial port or motor board comm issue.")

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        try:
            self.bot.disconnect()
        except Exception:
            pass


def main():
    d = MotorDriver()

    try:
        if ENABLE_STARTUP_MOTOR_TEST:
            d.startup_test()

        if not RUN_LINE_FOLLOWER:
            print("\nRUN_LINE_FOLLOWER=False, exiting after diagnostics.")
            return

        # (line follow code can be re-enabled later)
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)

        if not cap.isOpened():
            raise RuntimeError("Could not open camera")

        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            cv2.imshow("Debug", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

    finally:
        d.close()


if __name__ == "__main__":
    main()
