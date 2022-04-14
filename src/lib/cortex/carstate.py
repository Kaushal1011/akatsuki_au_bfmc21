import datetime
import math


class DetectedSign:
    def init(self):
        self.priority = False
        self.crosswalk = False
        self.stop = False
        self.roundabout = False
        self.parking = False
        self.highway_exit = False
        self.oneway = False
        self.noentry = False
        self.highway_entry = False

    def asdict(self):
        return {
            "priority": self.priority,
            "crosswalk": self.crosswalk,
            "stop": self.stop,
            "roundabout": self.roundabout,
            "parking": self.parking,
            "highway_exit": self.highway_exit,
            "oneway": self.oneway,
            "noentry": self.noentry,
            "highway_entry": self.highway_entry,
        }


class CarState:
    def __init__(self, max_v=0.1, dt=0.13, car_len=0.365) -> None:

        self.max_v = max_v
        # position data
        # 0.75, 4.8
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        # lane keeping
        self.lanekeeping_angle = 0.0

        # intersection detected
        self.detected_intersection = False

        # sign detection
        self.detected_sign = DetectedSign()

        # distance sensor
        self.front_distance = float("inf")
        self.side_distance = float("inf")

        # object detection
        self.detected_car = False
        self.detected_closed_road = False
        self.detected_pedestrian = False

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

        self.car_len = car_len

    def calc_distance(self, point_x: float, point_y: float) -> float:
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

    def check_goal_reached(self) -> bool:
        return self.calc_distance(*self.goal) < 0.01

    def update_pos(
        self, x: float, y: float, yaw: float, pitch: float, roll: float
    ) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

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
        self.detected_sign.priority = priority
        self.detected_sign.crosswalk = crosswalk
        self.detected_sign.stop = stop
        self.detected_sign.roundabout = roundabout
        self.detected_sign.parking = parking
        self.detected_sign.highway_exit = highway_exit
        self.detected_sign.oneway = oneway
        self.detected_sign.noentry = noentry
        self.detected_sign.highway_entry = highway_entry

    def update_tl(self, tl):
        self.tl = tl

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle:.4f}, {self.det_intersection}, {self.x:.4f}, {self.y:.4f}, {self.yaw:.4f}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}
