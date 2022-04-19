# Libraries
import RPi.GPIO as GPIO
import time

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER_1 = 20
GPIO_ECHO_1 = 21
GPIO_TRIGGER_2 = 17
GPIO_ECHO_2 = 27

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
GPIO.setup(GPIO_ECHO_1, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
GPIO.setup(GPIO_ECHO_2, GPIO.IN)


def distance_1():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER_1, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_1, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO_1) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO_1) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance


def distance_2():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER_2, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_2, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO_2) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO_2) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance


if __name__ == "_main_":
    try:
        while True:
            dist_1 = distance_1()
            dist_2 = distance_2()
            print("Measured Distance 1 = %.1f cm" % dist_1)
            print("Measured Distance 2 = %.1f cm" % dist_2)
            time.sleep(0.1)
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()