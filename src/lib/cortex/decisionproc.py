import datetime
import json
import math
from multiprocessing import Pipe
from multiprocessing.connection import Connection
import pathlib
from threading import Thread
from time import time
from time import sleep
from typing import Dict, List, Optional, Tuple
import socket

from src.config import get_config

config = get_config()

from src.lib.cortex.carstate import CarState
from src.lib.perception.signdetection import loaded_model
from src.templates.workerprocess import WorkerProcess
from time import time
from src.lib.cortex.action import (
    ActionBehaviour,
    ActionManager,
    CrosswalkBehavior,
    LaneKeepBehaviour,
    ControlSystemBehaviour,
    ObjectStopBehaviour,
    ParkingBehaviour,
    RoadBlocked,
    StopBehvaiour,
    PriorityBehaviour,
    OvertakeBehaviour,
    RoundAboutBehaviour,
    TLBehaviour,
)
import joblib
from loguru import logger
import math

import zmq

rx = []
ry = []


def trigger_behaviour(carstate: CarState, action_man: ActionManager):
    triggerparking = False

    if hasattr(carstate, "parkingcoords"):
        # print("Triggered Parking")
        d = math.sqrt(
            (carstate.rear_x - carstate.parkingcoords[0]) ** 2
            + (carstate.rear_y - carstate.parkingcoords[1]) ** 2
        )
        if d < 0.2:
            triggerparking = True

    # print(carstate.side_distance)
    if carstate.detected_intersection and carstate.current_ptype == "int":
        # intersection
        pass

    if carstate.detected_intersection:
        # pass
        # stop for t secs intersection
        stopobj = StopBehvaiour()
        stopaction = ActionBehaviour(name="stop", release_time=6.0, callback=stopobj)
        action_man.set_action(stopaction, action_time=3.0)

    if carstate.detected["parking"][0]:
        print("In parking trigger: ", triggerparking, carstate.detected["parking"][0])
        # Parking
        parkobj = ParkingBehaviour(car_state=carstate)
        parkobjaction = ActionBehaviour(name="parking", callback=parkobj)
        action_man.set_action(parkobjaction, action_time=None, car_state=carstate)

    # print("carstate-frnt: ", carstate.front_distance)
    # print("carstate-can-overtake:", carstate.can_overtake)
    if (
        carstate.front_distance < 0.7
        and carstate.detected["car"][0]
        and carstate.can_overtake
    ):
        print("Overtake Trigger")
        # overtake
        overtakeobj = OvertakeBehaviour(car_state=carstate)
        overtakeobjaction = ActionBehaviour(name="overtaking", callback=overtakeobj)
        action_man.set_action(overtakeobjaction, action_time=None, car_state=carstate)

    if (
        carstate.front_distance < 0.6
        and carstate.detected["car"][0]
        and not carstate.can_overtake
    ):
        # tailing or stop
        pass

    if carstate.detected["roadblock"][0] and carstate.front_distance < 0.45:  # 10 cm
        # replan closed road
        robobj = RoadBlocked(car_state=carstate)
        robobjaction = ActionBehaviour(name="roadblocked", callback=robobj)
        action_man.set_action(robobjaction, action_time=None, car_state=carstate)

    if carstate.detected["roundabout"][0] or (
        carstate.current_ptype == "roundabout" and action_man.l1_ab is None
    ):
        print("In round about trigger: ", carstate.current_ptype)
        rabobj = RoundAboutBehaviour(car_state=carstate)
        rabobjaction = ActionBehaviour(name="roundabout", callback=rabobj)
        action_man.set_action(rabobjaction, action_time=None, car_state=carstate)

    if carstate.detected["stop"][0]:
        # stop for t secs
        stopobj = StopBehvaiour()
        stopaction = ActionBehaviour(name="stop", release_time=6.0, callback=stopobj)
        action_man.set_action(stopaction, action_time=3.0)

    if carstate.detected["priority"][0]:
        pass
        # slowdown for t secs
        # priorityobj = PriorityBehaviour()
        # priorityaction = ActionBehaviour(
        #     name="priority", release_time=6.0, callback=priorityobj
        # )
        # action_man.set_action(priorityaction, action_time=9.0)

    if carstate.detected["crosswalk"][0]:
        # stop detected pedestrain or crosswalk
        cwobj = CrosswalkBehavior(car_state=carstate)
        cwobjaction = ActionBehaviour(name="crosswalk", callback=cwobj)
        action_man.set_action(cwobjaction, action_time=None, car_state=carstate)

    if carstate.detected["trafficlight"][0]:
        tlobj = TLBehaviour(carstate=carstate)
        tlaction = ActionBehaviour(name="trafficlight", callback=tlobj)
        action_man.set_action(tlaction, action_time=None, carstate=carstate)

    if carstate.pitch > 0.2:
        # incline trigger ramp
        pass

    if carstate.pitch < -0.2:
        # decline exit ramp
        pass


ENVID = {
    "car": 10,
    "crosswalk": 4,
    "highway_entry": 5,
    "highway_exit": 6,
    "no_entry": 1,
    "onewayroad": 8,
    "parking": 3,
    "pedestrian": 11,
    "priority": 2,
    "roadblock": 14,
    "roundabout": 7,
    "stop": 1,
    "trafficlight": 9,
}


def send_data2env(car_state: CarState, detections: dict):
    x = car_state.x
    y = car_state.y
    return [
        {"obstacle_id": ENVID[c], "x": x, "y": y}
        for c in detections.keys()
        if not car_state.detected[c][0]
    ]


def get_last(socket):
    data = socket.recv_json()
    count = 0
    while socket.poll(timeout=0.05):
        data = socket.recv_json()
        count += 1
        if count == 10:
            break
    return data


class DecisionMakingProcess(WorkerProcess):
    # ===================================== Worker process =========================================
    def __init__(self, inPs, outPs, inPsnames=[]):
        """Process used for the image processing needed for lane keeping and for computing the steering value.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes (0 - receive image feed from the camera)
        outPs : list(Pipe)
            List of output pipes (0 - send steering data to the movvement control process)
        """
        super(DecisionMakingProcess, self).__init__(inPs, outPs)
        # pass navigator config
        self.state = CarState()
        self.inPsnames = inPsnames
        self.actman = ActionManager()

        lkobj = LaneKeepBehaviour()
        lkaction = ActionBehaviour(name="lk", callback=lkobj)
        self.actman.set_action(lkaction)

        ##################################################################
        # data_path = pathlib.Path(
        #     pathlib.Path(__file__).parent.parent.parent.resolve(),
        #     "data",
        #     "new_course.z",
        # )
        # data = joblib.load(data_path)
        # cx = data["x"]
        # cy = data["y"]
        # coord_list = [x for x in zip(cx, cy)]
        # # coord_list = data[0]
        #################################################################

        # pass coordlist here from navigator config
        csobj = ControlSystemBehaviour(coord_list=self.state.navigator.coords)
        # csobj = ControlSystemBehaviour(coord_list=coord_list)

        csaction = ActionBehaviour(name="cs", callback=csobj)
        self.actman.set_action(csaction)

        # pass coordlist here from navigator config
        stopobj = ObjectStopBehaviour()
        stopobjaction = ActionBehaviour(name="objstop", callback=stopobj)
        self.actman.set_action(stopobjaction)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(DecisionMakingProcess, self).run()

    def _init_threads(self):
        """Initialize the thread."""
        if self._blocker.is_set():
            return

        thr = Thread(
            name="DecisionMakingThread",
            target=self._the_thread,
            args=(
                self.inPs,
                self.outPs,
            ),
        )
        thr.daemon = True
        self.threads.append(thr)

    def _the_thread(self, inPs: List[Connection], outPs: List[Connection]):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        if "lk" in self.inPsnames:
            context_recv_lk = zmq.Context()
            sub_lk = context_recv_lk.socket(zmq.SUB)
            # sub_lk.setsockopt(zmq.CONFLATE, 1)
            sub_lk.connect("ipc:///tmp/v51")
            sub_lk.setsockopt_string(zmq.SUBSCRIBE, "")

        if "sd" in self.inPsnames:
            context_recv_sd = zmq.Context()
            sub_sd = context_recv_sd.socket(zmq.SUB)
            # sub_sd.setsockopt(zmq.CONFLATE, 1)
            sub_sd.connect("ipc:///tmp/v61")
            sub_sd.setsockopt_string(zmq.SUBSCRIBE, "")

        if "pos" in self.inPsnames:
            context_recv_pos = zmq.Context()
            sub_pos = context_recv_pos.socket(zmq.SUB)
            # sub_pos.setsockopt(zmq.CONFLATE, 1)
            sub_pos.connect("ipc:///tmp/v42")
            sub_pos.setsockopt_string(zmq.SUBSCRIBE, "")

        if "dis" in self.inPsnames:
            context_recv_dis = zmq.Context()
            sub_dis = context_recv_dis.socket(zmq.SUB)
            # sub_dis.setsockopt(zmq.CONFLATE, 1)
            sub_dis.connect("ipc:///tmp/v11")
            sub_dis.setsockopt_string(zmq.SUBSCRIBE, "")

        if "tl" in self.inPsnames:
            context_recv_tl = zmq.Context()
            sub_tl = context_recv_tl.socket(zmq.SUB)
            # sub_dis.setsockopt(zmq.CONFLATE, 1)
            sub_tl.connect("ipc:///tmp/vtl")
            sub_tl.setsockopt_string(zmq.SUBSCRIBE, "")

        while True:
            try:
                # c = time()
                # start_time = time()
                # t_lk = time()
                if "lk" in self.inPsnames:
                    if sub_lk.poll(timeout=0.05):
                        lk_angle, detected_intersection = get_last(sub_lk)
                        logger.log(
                            "PIPE",
                            f"LK -> Angle {lk_angle} Intersection Det {detected_intersection}",
                        )
                        print("LK -> ", lk_angle, detected_intersection)
                        self.state.update_lk_angle(lk_angle)
                        self.state.update_intersection(detected_intersection)
                        print("LK angle", self.state.lanekeeping_angle)
                # logger.log("PIPE", f"Recv->LK {lk_angle}")
                # logger.log("SYNC", f"LK timedelta {time()- lk_timestamp}")
                # # print(f"Time taken lk {(time() - t_lk):.4f}s {lk_angle}")

                # sign Detection
                # TODO
                # t_sD = time()
                if "dis" in self.inPsnames:
                    if sub_dis.poll(timeout=0.05):
                        distance_data = get_last(sub_dis)
                        logger.log(
                            "PIPE",
                            f"DIS -> {distance_data}",
                        )
                        print("DIS -> ", distance_data)
                        logger.log(
                            "SYNC", f"dis delta {time()- distance_data['timestamp']}"
                        )
                        self.state.update_object_det(
                            distance_data["sonar1"], distance_data["sonar2"]
                        )

                if "pos" in self.inPsnames:
                    if sub_pos.poll(timeout=0.05):
                        pos = get_last(sub_pos)
                        logger.log(
                            "PIPE",
                            f"POS -> {pos}",
                        )
                        # print(f"POS -> {pos}")
                        # print(pos["timestamp"] - time.time())
                        if pos[0] == 0 and pos[1] == 0:
                            pass
                        else:
                            self.state.update_pos(*pos)

                    else:
                        self.state.update_pos_noloc()

                if "sd" in self.inPsnames:
                    if sub_sd.poll(timeout=0.05):
                        detections = get_last(sub_sd)
                        logger.log("PIPE", f"SD -> {detections}")
                        print("SD ->", detections)
                        # send data to env server
                        if len(outPs) > 1:
                            for env_data in send_data2env(self.state, detections):
                                print("ENV -> ", env_data)
                                logger.log("PIPE", f"ENV -> {env_data}")
                                outPs[1].send(env_data)

                        self.state.update_detected(detections)
                #         print("SD <-<", sign)
                #         logger.log("SYNC", f"SD timedelta {time() - sd_timestamp}")
                #         logger.log("PIPE", f"Recv -> SD {sign}")

                if "tl" in self.inPsnames:
                    if sub_tl.poll(timeout=0.05):
                        tl_data = sub_tl.recv()
                        logger.log("PIPE", f"TL ->  {tl_data}")

                        print(f"TL -> {tl_data}")

                        self.state.update_tl(tl_data)

                # # update car navigator, current ptype, current etype and current idx

                trigger_behaviour(self.state, self.actman)
                # print(self.state.detected)
                speed, steer = self.actman(self.state)
                self.state.v = speed
                self.state.steering_angle = steer
                rx.append(self.state.x)
                ry.append(self.state.y)
                logger.log("XY", f"{self.state.x}, {self.state.y},")
                logger.debug(f"Sonar Front: {self.state.front_distance}")
                logger.debug(f"Sonar Side: {self.state.side_distance}")
                print("OUT",self.state.steering_angle, self.state.v)
                if len(outPs) > 0:
                    # outPs[0].send((self.state.steering_angle, self.state.v))
                    outPs[0].send((0.0, 0.0))
                sleep(0.1)

            except Exception as e:
                print("Decision Process error:")
                raise e
            finally:
                pass
                # joblib.dump({"x": rx, "y": ry}, "real_coords.z")
