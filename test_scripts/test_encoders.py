import RPi.GPIO as GPIO
from time import sleep

# Define the encoder pin
encoder_pinA = 14  # Replace with the pin number you are using
encoder_pinB = 15  # Replace with the pin number you are using


pulses_per_rev = 12 # Specified by manufacturer

def accuracyTest():
    # Initialize global variables
    encoder_pinA_last = None
    pulses = 0

    def wheelSpeed(channel):
        nonlocal pulses
        if (GPIO.input(encoder_pinB) != GPIO.input(encoder_pinA)): # B edge has not risen yet. B comes after A in clockwise motion
            pulses += 1 
        else:                                    # B edge has already risen. B comes before A in counter clockwise motion
            pulses -= 1

    def setupPins():
        print("SETUP")

        nonlocal encoder_pinA_last
        # Use BCM pin numbering
        GPIO.setmode(GPIO.BCM)
        # Set up the pin as an input with an internal pull-up resistor
        # GPIO.setup(encoder_pinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # GPIO.setup(encoder_pinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(encoder_pinA, GPIO.IN)
        GPIO.setup(encoder_pinB, GPIO.IN)
        
        GPIO.add_event_detect(encoder_pinA, GPIO.RISING)  # add rising edge detection on a channel
        GPIO.add_event_callback(encoder_pinA, wheelSpeed)

        encoder_pinA_last = GPIO.input(encoder_pinA)

    setupPins()

    try:
        while True:
            # if (abs(pulses) == pulses_per_rev):
            #     print("ONE ROTATION")
            #     break
            print(pulses)
            sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        GPIO.cleanup()

# Check if the motor encoders are being set to HIGH and LOW correctly
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

            print(f"A state: {'HIGH' if pin_stateA else 'LOW'}" + ", " + f"B state: {'HIGH' if pin_stateB else 'LOW'}")
            sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting program")

    # Clean up GPIO settings before exiting
    GPIO.cleanup()

# senseTest()
accuracyTest()