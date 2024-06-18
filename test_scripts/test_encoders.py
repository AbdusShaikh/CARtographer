import RPi.GPIO as GPIO
from time import sleep

# Define the encoder pin
encoder_pinA = 18  # Replace with the pin number you are using
encoder_pinB = 23  # Replace with the pin number you are using


def accuracyTest():
    encoder_pinA_last = GPIO.LOW
    direction = False
    duration = 0
    def wheelSpeed(channel):
        Lstate = GPIO.input(encoder_pinA)
        if (encoder_pinA_last == GPIO.LOW and Lstate == GPIO.HIGH):
            bVal = GPIO.input(encoder_pinB)
            if (bVal == GPIO.LOW and direction):
                direction = False
        elif (bVal == GPIO.HIGH and not direction):
                direction = True
        encoder_pinA_last = Lstate

        if (not direction): duration += 1
        else: direction -= 1

    def setupPins():
        print("SETUP")
        # Use BCM pin numbering
        GPIO.setmode(GPIO.BCM)
        # Set up the pin as an input with an internal pull-up resistor
        GPIO.setup(encoder_pinA, GPIO.IN)
        GPIO.setup(encoder_pinB, GPIO.IN)
        
        GPIO.add_event_detect(encoder_pinA, GPIO.BOTH)  # add rising edge detection on a channel
        # GPIO.add_event_detect(encoder_pinB, GPIO.BOTH)  # add rising edge detection on a channel

        GPIO.add_event_callback(encoder_pinA, wheelSpeed)

    setupPins()
    while True:
        print("Pulses:", duration)
        duration = 0
        sleep(0.1)

def senseTest():
    # Use BCM pin numbering
    GPIO.setmode(GPIO.BCM)
    # Set up the pin as an input with an internal pull-up resistor
    GPIO.setup(encoder_pinA, GPIO.IN)
    GPIO.setup(encoder_pinB, GPIO.IN)

    try:
        print("Monitoring encoder pin. Rotate the wheel and observe the output.")
        while True:
            # Read the pin state
            pin_stateA = GPIO.input(encoder_pinA)
            pin_stateB = GPIO.input(encoder_pinB)

            print(f"Pin state: {'HIGH' if pin_stateA else 'LOW'}" + ", " + f"Pin state: {'HIGH' if pin_stateB else 'LOW'}")
            sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting program")

    # Clean up GPIO settings before exiting
    GPIO.cleanup()

accuracyTest()