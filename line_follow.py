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
    motor_dead = False   # <--- add this

    print("Starting line follower. Press 'q' in display window or Ctrl+C to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                if not motor_dead:
                    try:
                        driver.stop()
                    except Exception:
                        motor_dead = True
                time.sleep(0.05)
                continue

            error, debug, _ = find_line_error(frame)

            if error is None:
                if time.time() - last_seen_time > LOST_LINE_TIMEOUT_S and not motor_dead:
                    try:
                        driver.stop()
                    except Exception as e:
                        print(f"Motor communication failed while stopping: {e}")
                        motor_dead = True
                cv2.putText(
                    debug, "LINE LOST", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA
                )
            else:
                last_seen_time = time.time()
                steer = pid.update(error)
                steer = float(np.clip(steer, -1.0, 1.0))

                if not motor_dead:   # <--- critical guard
                    try:
                        apply_steering(driver, BASE_SPEED_PERCENT, steer)
                    except Exception as e:
                        print(f"Motor communication failed: {e}")
                        print("Disabling motor commands for this run.")
                        motor_dead = True

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
        if not motor_dead:
            try:
                driver.stop()
            except Exception:
                pass
        cap.release()
        cv2.destroyAllWindows()
        print("Stopped.")
