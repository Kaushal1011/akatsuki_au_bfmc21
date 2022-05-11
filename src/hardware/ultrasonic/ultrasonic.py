from typing import List
from loguru import logger

try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError as e:
    logger.error(e)
    pass

from multiprocessing.connection import Connection
import time
import threading
import zmq

# set GPIO Pins
GPIO_TRIGGER_FRONT = 20
GPIO_ECHO_FRONT = 21

GPIO_TRIGGER_SIDE = 27
GPIO_ECHO_SIDE = 17


class Ultrasonic(threading.Thread):
    def __init__(self, outPs: List[Connection]):
        threading.Thread.__init__(self)
        self.running = True
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # set GPIO direction (IN / OUT)
        GPIO.setup(GPIO_TRIGGER_FRONT, GPIO.OUT)
        GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)
        GPIO.setup(GPIO_TRIGGER_SIDE, GPIO.OUT)
        GPIO.setup(GPIO_ECHO_SIDE, GPIO.IN)

        self.outPs = outPs

    @staticmethod
    def get_distance(GPIO_TRIGGER: int, GPIO_ECHO: int) -> float:
        GPIO.setmode(GPIO.BCM)
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
        distance = (TimeElapsed * 34300) / 2.0

        return distance / 100

    def run(self):
        context_send = zmq.Context()
        pub_dis = context_send.socket(zmq.PUB)
        pub_dis.bind(f"ipc:///tmp/v11")

        while True:
            sonar1 = self.get_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT)
            time.sleep(0.3)
            sonar2 = self.get_distance(GPIO_TRIGGER_SIDE, GPIO_ECHO_SIDE)
            # print("Measured Distance front = %.4f m" % sonar1)
            # print("Measured Distance side = %.4f m" % sonar2)
            time.sleep(0.3)
            data = {"timestamp": time.time(), "sonar1": sonar1, "sonar2": sonar2}
            pub_dis.send_json(data, flags=zmq.NOBLOCK)

    def stop(self):
        print("IN STOP")
        self.running = False
        GPIO.cleanup()


if __name__ == "__main__":
    try:
        ultrasonic = Ultrasonic([])
        while True:
            dist_1 = ultrasonic.get_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT)
            time.sleep(0.3)
            dist_2 = ultrasonic.get_distance(GPIO_TRIGGER_SIDE, GPIO_ECHO_SIDE)

            print("Measured Distance front = %.4f m" % dist_1)
            print("Measured Distance side = %.4f m" % dist_2)
            time.sleep(0.3)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
