import datetime
import math

from time import time
from src.lib.cortex.navigation import Navigator

activity_config={
    "nodes":[[86,99],[100,145],[61,168],[169,229],[230,104],[105,465],[466,85]],
    "activity":["navigation","roadblocked","parking","overtaking","highway","oneway","finish"]
}

class CarState:
    def __init__(self, max_v=0.20, dt=0.13, car_len=0.365, **kwargs) -> None:

        self.max_v = max_v
        # position data
        # 0.75, 4.8
        self.x = 0.8
        self.y = 14.8
        self.yaw = 1.57
        self.pitch = 0
        self.roll = 0
        self.car_len = car_len
        self.rear_x = self.x - ((car_len / 2) * math.cos(-self.yaw))
        self.rear_y = self.y - ((car_len / 2) * math.sin(-self.yaw))

        self.target_x = None
        self.target_y = None

        self.navigator = Navigator(activity_config)
        # plan path -> self.navigator.plan_path(self.x,self.y,self.yaw)
        # current node -> self.navigator.get_current_node(self.x, self.y, self.yaw)
        self.last_update_time = time()

        # lane keeping and cs
        self.lanekeeping_angle = 0.0
        self.cs_angle = 0.0

        # intersection detected
        self.detected_intersection = False

        self.parkingcoords=(2.94,2.09)

        # sign detection
        self.detected_sign = {
            "priority": False,
            "crosswalk": False,
            "stop": False,
            "roundabout": False,
            "parking": False,
            "highway_exit": False,
            "oneway": False,
            "noentry": False,
            "highway_entry": False,
        }

        # distance sensor
        self.front_distance = float("inf")
        self.side_distance = float("inf")

        # object detection
        self.detected_car = False
        self.detected_closed_road = False
        self.detected_pedestrian = False
        self.target_ind=None
        # traffic light semaphore
        self.tl = {}

        # active behavious
        self.active_behavious = []

        # navigation (control sys)
        self.current_target = (0, 0)
        self.can_overtake = False
        self.current_ptype = ""

        # goal
        self.goal = (float("inf"), float("inf"))
        # control parameters
        self.steering_angle = 0.0
        self.v = max_v
        self.priority_speed=0.1
        self.highway_speed = 0.25

        # activity type

        self.activity_type = None
        self.car_len = car_len

    def calc_distance(self, point_x: float, point_y: float) -> float:
        dx = self.x - point_x
        dy = self.y - point_y
        # print(dx,dy)
        return math.hypot(dx, dy)

    def check_goal_reached(self) -> bool:
        return self.calc_distance(*self.goal) < 0.01

    def calc_distance_target_node(self) -> float:
        return self.calc_distance(*self.current_target)

    def update_pos(
        self, x: float, y: float, yaw: float, pitch: float, roll: float
    ) -> None:
        self.last_update_time = time()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.rear_x = self.x - ((self.car_len / 2) * math.cos(-self.yaw))
        self.rear_y = self.y - ((self.car_len / 2) * math.sin(-self.yaw))

    def update_pos_noloc(self):
        # Use Of Inverted Yaw Here
        dt = time() - self.last_update_time
        self.last_update_time = time()
        self.x = self.x + self.v * math.cos(-self.yaw) * dt
        self.y = self.y + self.v * math.sin(-self.yaw) * dt
        self.yaw = (
            self.yaw
            - self.v / self.car_len * math.tan(self.steering_angle * math.pi / 180) * dt
        )
        self.rear_x = self.x - ((self.car_len / 2) * math.cos(-self.yaw))
        self.rear_y = self.y - ((self.car_len / 2) * math.sin(-self.yaw))

    def update_intersection(self, detected_intersection: bool) -> None:
        self.detected_intersection = detected_intersection

    def update_lk_angle(self, lk_angle: float) -> None:
        self.lanekeeping_angle = lk_angle

    def update_object_det(
        self,
        front_distance: float,
        side_distance: float,
        detected_car: bool,
        detected_pedestrain: bool,
        detected_closed_road: bool,
    ) -> None:
        self.front_distance = front_distance
        self.side_distance = side_distance
        self.detected_car = detected_car
        self.detected_pedestrian = detected_pedestrain
        self.detected_closed_road = detected_closed_road

    def update_sign_detected(
        self,
        priority: bool,
        crosswalk: bool,
        stop: bool,
        roundabout: bool,
        parking: bool,
        highway_exit: bool,
        oneway: bool,
        noentry: bool,
        highway_entry: bool,
    ):
        self.detected_sign["priority"] = priority
        self.detected_sign["crosswalk"] = crosswalk
        self.detected_sign["stop"] = stop
        self.detected_sign["roundabout"] = roundabout
        self.detected_sign["parking"] = parking
        self.detected_sign["highway_exit"] = highway_exit
        self.detected_sign["oneway"] = oneway
        self.detected_sign["noentry"] = noentry
        self.detected_sign["highway_entry"] = highway_entry

    def update_tl(self, tl):
        self.tl = tl

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle:.4f}, {self.det_intersection}, {self.x:.4f}, {self.y:.4f}, {self.yaw:.4f}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}
