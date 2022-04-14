import datetime
import math
from multiprocessing import Pipe
from threading import Thread
from time import time
from typing import Optional
from src.templates.workerprocess import WorkerProcess
from time import time
import math

class CarState:
    def __init__(self, max_v=0.1, dt=0.13, car_len=0.365) -> None:

        # TODO: get initial position from config IDK
        
        self.max_v = max_v
        # 0.75, 4.8
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.pitch=0
        self.roll=0

        # control parameters
        self.steering_angle=0.0
        self.v = max_v

        self.tl = {}
        
        self.dt = dt
        self.car_len = car_len
        self.closest_pt = None
        self.last_update_time = time()

    def update_pos(self):
        dt = time() - self.last_update_time
        self.last_update_time = time()
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.car_len * math.tan(self.steering_angle) * dt

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

    def update(
        self,
        x: float,
        y: float,
        yaw: float,
        pitch: float,
        roll:float
    ) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle:.4f}, {self.det_intersection}, {self.x:.4f}, {self.y:.4f}, {self.yaw:.4f}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}
