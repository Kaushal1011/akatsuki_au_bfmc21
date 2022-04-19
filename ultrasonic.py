import RPi.GPIO as GPIO
import time
import functools

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER_FRONT = 20
GPIO_ECHO_FRONT = 21

GPIO_TRIGGER_SIDE = 27
GPIO_ECHO_SIDE = 17

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER_FRONT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_SIDE, GPIO.OUT)
GPIO.setup(GPIO_ECHO_SIDE, GPIO.IN)


def distance(GPIO_TRIGGER, GPIO_ECHO):
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance


distance_front = functools.partial(distance, GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT)
distance_side = functools.partial(distance, GPIO_TRIGGER_SIDE, GPIO_ECHO_SIDE)

if __name__ == "__main__":
    try:
        while True:
            dist_1 = distance_front()
            dist_2 = distance_side()
            print("Measured Distance front = %.1f cm" % dist_1)
            print("Measured Distance side = %.1f cm" % dist_2)

            time.sleep(0.5)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
