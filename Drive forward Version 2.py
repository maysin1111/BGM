import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# -----------------------------
# Motor Pin Configuration
# (Replace with your own GPIO pins)
# -----------------------------

FL_IN1, FL_IN2, FL_PWM = 5, 6, 12
FR_IN1, FR_IN2, FR_PWM = 13, 19, 18
RL_IN1, RL_IN2, RL_PWM = 16, 20, 21
RR_IN1, RR_IN2, RR_PWM = 23, 24, 25

pins = [
    FL_IN1, FL_IN2, FL_PWM,
    FR_IN1, FR_IN2, FR_PWM,
    RL_IN1, RL_IN2, RL_PWM,
    RR_IN1, RR_IN2, RR_PWM
]

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

# PWM Setup
fl_pwm = GPIO.PWM(FL_PWM, 1000)
fr_pwm = GPIO.PWM(FR_PWM, 1000)
rl_pwm = GPIO.PWM(RL_PWM, 1000)
rr_pwm = GPIO.PWM(RR_PWM, 1000)

for pwm in [fl_pwm, fr_pwm, rl_pwm, rr_pwm]:
    pwm.start(0)


def drive_motor(in1, in2, pwm, speed):
    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(speed)
    elif speed < 0:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(abs(speed))
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)


try:
    speed = 60  # PWM duty cycle (0-100%)

    # Move all four motors forward
    drive_motor(FL_IN1, FL_IN2, fl_pwm, speed)
    drive_motor(FR_IN1, FR_IN2, fr_pwm, speed)
    drive_motor(RL_IN1, RL_IN2, rl_pwm, speed)
    drive_motor(RR_IN1, RR_IN2, rr_pwm, speed)

    print("Moving forward...")
    sleep(5)

    # Stop all motors
    drive_motor(FL_IN1, FL_IN2, fl_pwm, 0)
    drive_motor(FR_IN1, FR_IN2, fr_pwm, 0)
    drive_motor(RL_IN1, RL_IN2, rl_pwm, 0)
    drive_motor(RR_IN1, RR_IN2, rr_pwm, 0)

    print("Stopped.")

finally:
    fl_pwm.stop()
    fr_pwm.stop()
    rl_pwm.stop()
    rr_pwm.stop()
    GPIO.cleanup()
