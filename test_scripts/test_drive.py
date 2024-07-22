from gpiozero import Motor
from time import sleep
import RPi.GPIO as GPIO

# rightMotor = Motor(forward=20, backward=16)
# leftMotor = Motor(forward=19, backward=13)
# try:
#     # Turn the motor on
#     rightMotor.forward()
#     leftMotor.forward()
#     sleep(5)
#     rightMotor.backward()
#     leftMotor.backward()
#     sleep(5)
#     rightMotor.stop()
#     leftMotor.stop()
# except:
#     print("FAILED")
# print ("SUCCESS")


import RPi.GPIO as GPIO
import time

# Pin definitions
ENA = 21   # Enable pin for Motor A
IN1 = 20   # Control pin 1 for Motor A (Forward)
IN2 = 16   # Control pin 2 for Motor A
ENB = 26   # Enable pin for Motor B
IN3 = 19   # Control pin 1 for Motor B (Forward)
IN4 = 13    # Control pin 2 for Motor B

sleep_time = 1


def test_drive():
    # GPIO setup
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    GPIO.setup(ENA, GPIO.OUT)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(ENB, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)

    # Set up PWM on the enable pins
    pwmA = GPIO.PWM(ENA, 1000)  # 1000 Hz frequency
    pwmB = GPIO.PWM(ENB, 1000)  # 1000 Hz frequency
    pwmA.start(100)  # Start PWM with 100% duty cycle (full speed)
    pwmB.start(100)  # Start PWM with 100% duty cycle (full speed)

    def motorA_forward():
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)

    def motorA_backward():
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)

    def motorB_forward():
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

    def motorB_backward():
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)

    def motorA_stop():
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    def motorB_stop():
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)

    try:
        print("Motors forward")
        motorA_forward()
        motorB_forward()
        time.sleep(sleep_time)

        print("Motors backward")
        motorA_backward()
        motorB_backward()
        time.sleep(sleep_time)

        print("Stop motors")
        motorA_stop()
        motorB_stop()

    except KeyboardInterrupt:
        print("Process interrupted")

    finally:
        motorA_stop()
        motorB_stop()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()

test_drive()