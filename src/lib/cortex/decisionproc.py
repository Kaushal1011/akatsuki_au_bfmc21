import datetime
import math
from multiprocessing import Pipe
from multiprocessing.connection import Connection
import pathlib
from threading import Thread
from time import time
from typing import List, Optional


from src.lib.cortex.carstate import CarState
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
    StopBehvaiour,
    PriorityBehaviour,
    OvertakeBehaviour,
)
import joblib
from loguru import logger
import math

import zmq

rx = []
ry = []


def get_last(inP: Pipe, delta_time: float = 1e-2):
    timestamp, data = inP.recv()
    while (time() - timestamp) > delta_time:
        timestamp, data = inP.recv()
    return timestamp, data


def get_last_value(inP: Connection, required: bool = True):
    timestamp, data = inP.recv()

    while inP.poll():
        timestamp, data = inP.recv()
    return timestamp, data


def get_last_lk_id(inP: Connection):
    timestamp, lk, id = inP.recv()

    while inP.poll():
        timestamp, lk, id = inP.recv()
    return timestamp, lk, id


def get_last_distance(inP: Connection):
    data = inP.recv()
    while inP.poll():
        data = inP.recv()
    return data


def trigger_behaviour(carstate: CarState, action_man: ActionManager):
    triggerparking = False

    if hasattr(carstate, "parkingcoords"):
        print("Triggered Parking")
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

    if carstate.detected_sign["parking"] or triggerparking:
        print("In parking trigger: ", triggerparking, carstate.detected_sign["parking"])
        # Parking
        # parkobj = ParkingBehaviour(car_state=carstate)
        # parkobjaction = ActionBehaviour(name="parking", callback=parkobj)
        # action_man.set_action(parkobjaction, action_time=None, car_state=carstate)

    if (
        carstate.front_distance
        < 0.5
        # and carstate.detected_car
        # and carstate.can_overtake
    ):
        # print("Overtake Trigger")
        # overtake
        overtakeobj = OvertakeBehaviour(car_state=carstate)
        overtakeobjaction = ActionBehaviour(name="overtaking", callback=overtakeobj)
        action_man.set_action(overtakeobjaction, action_time=None, car_state=carstate)

    if (
        carstate.front_distance < 0.6
        and carstate.detected_car
        and not carstate.can_overtake
    ):
        # tailing or stop
        pass

    if (
        carstate.detected_closed_road or carstate.calc_distance_target_node() > 2
    ):  # 10 cm
        # replan closed road
        pass

    if carstate.detected_sign["stop"]:
        # stop for t secs
        stopobj = StopBehvaiour()
        stopaction = ActionBehaviour(name="stop", release_time=6.0, callback=stopobj)
        action_man.set_action(stopaction, action_time=3.0)

    if carstate.detected_sign["priority"]:
        # slowdown for t secs
        priorityobj = PriorityBehaviour()
        priorityaction = ActionBehaviour(
            name="priority", release_time=6.0, callback=priorityobj
        )
        action_man.set_action(priorityaction, action_time=9.0)

    if carstate.detected_sign["crosswalk"]:
        # stop detected pedestrain or crosswalk
        cwobj = CrosswalkBehavior(car_state=carstate)
        cwobjaction = ActionBehaviour(name="crosswalk", callback=cwobj)
        action_man.set_action(cwobjaction, action_time=None, car_state=carstate)

    if carstate.pitch > 0.2:
        # incline trigger ramp
        pass

    if carstate.pitch < -0.2:
        # decline exit ramp
        pass


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
        self.state = CarState(navigator_config=None)
        self.inPsnames = inPsnames
        self.actman = ActionManager()

        lkobj = LaneKeepBehaviour()
        lkaction = ActionBehaviour(name="lk", callback=lkobj)
        self.actman.set_action(lkaction)

        ##################################################################
        data_path = pathlib.Path(
            pathlib.Path(__file__).parent.parent.parent.resolve(),
            "data",
            "mid_course.z",
        )
        data = joblib.load(data_path)
        # cx = data["x"]
        # cy = data["y"]
        # coord_list = [x for x in zip(cx, cy)]
        coord_list = data[0]
        #################################################################

        # pass coordlist here from navigator config
        csobj = ControlSystemBehaviour(coord_list=coord_list)
        csaction = ActionBehaviour(name="cs", callback=csobj)
        self.actman.set_action(csaction)

        # pass coordlist here from navigator config
        # stopobj = ObjectStopBehaviour()
        # stopobjaction = ActionBehaviour(name="objstop", callback=stopobj)
        # self.actman.set_action(stopobjaction)

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
            sub_lk.setsockopt(zmq.CONFLATE, 1)
            sub_lk.connect("ipc:///tmp/v51")
            sub_lk.setsockopt_string(zmq.SUBSCRIBE, "")

        if "sd" in self.inPsnames:
            context_recv_sd = zmq.Context()
            sub_sd = context_recv_sd.socket(zmq.SUB)
            sub_sd.setsockopt(zmq.CONFLATE, 1)
            sub_sd.connect("ipc:///tmp/v61")
            sub_sd.setsockopt_string(zmq.SUBSCRIBE, "")

        if "pos" in self.inPsnames:
            context_recv_pos = zmq.Context()
            sub_pos = context_recv_pos.socket(zmq.SUB)
            sub_pos.setsockopt(zmq.CONFLATE, 1)
            sub_pos.connect("ipc:///tmp/v42")
            sub_pos.setsockopt_string(zmq.SUBSCRIBE, "")

        if "dis" in self.inPsnames:
            context_recv_dis = zmq.Context()
            sub_dis = context_recv_dis.socket(zmq.SUB)
            sub_dis.setsockopt(zmq.CONFLATE, 1)
            sub_dis.connect("ipc:///tmp/v11")
            sub_dis.setsockopt_string(zmq.SUBSCRIBE, "")

        while True:
            try:
                c = time()
                start_time = time()
                t_lk = time()
                if "lk" in self.inPsnames:
                    if sub_lk.poll(timeout=0.001):
                        lk_angle, detected_intersection = sub_lk.recv_json()
                        print("LK -> ", lk_angle, detected_intersection)
                        self.state.update_lk_angle(lk_angle)
                        self.state.update_intersection(detected_intersection)
                # logger.log("PIPE", f"Recv->LK {lk_angle}")
                # logger.log("SYNC", f"LK timedelta {time()- lk_timestamp}")
                # # print(f"Time taken lk {(time() - t_lk):.4f}s {lk_angle}")

                # sign Detection
                # TODO
                # t_sD = time()
                if "sd" in self.inPsnames:
                    if sub_sd.poll(timeout=0.001):
                        signs_data = sub_sd.recv_json()
                        print("SD ->", signs_data)
                #         print("SD <-<", sign)
                #         logger.log("SYNC", f"SD timedelta {time() - sd_timestamp}")
                #         logger.log("PIPE", f"Recv -> SD {sign}")

                #         # self.state.update_sign_detected()

                if "dis" in self.inPsnames:
                    if sub_sd.poll(timeout=0.01):
                        distance_data = sub_dis.recv_json()
                        final_data = (
                            distance_data["sonar1"],
                            distance_data["sonar2"],
                            False,
                            False,
                            False,
                        )
                        print("DIS -> ", distance_data["sonar1"], distance_data["sonar1"])
                        logger.log("PIPE", f"Recv->DIS {final_data[0]},{final_data[1]}")
                        logger.log(
                        "SYNC", f"dis delta {time()- distance_data['timestamp']}"
                        )
                        self.state.update_object_det(*final_data)

                if "pos" in self.inPsnames:
                    if sub_sd.poll(timeout=0.01):
                        pos = sub_pos.recv_json()
                        print(f"POS -> {pos}")
                        if pos[0] == 0 and pos[1] == 0:
                            pass
                        else:
                            self.state.update_pos(*pos)
                    else:
                        self.state.update_pos_noloc()

                # # if trafficlight process is connected
                # if "tl" in self.inPsnames:
                #     idx = self.inPsnames.index("tl")
                #     if inPs[idx].poll():
                #         trafficlights = inPs[idx].recv()
                #         print(trafficlights)

                # # update car navigator, current ptype, current etype and current idx

                # trigger_behaviour(self.state, self.actman)

                # speed, steer = self.actman(self.state)
                # self.state.v = speed
                # self.state.steering_angle = steer
                # rx.append(self.state.x)
                # ry.append(self.state.y)
                # logger.log("XY", f"{self.state.x}, {self.state.y},")
                # logger.debug(f"Sonar Front: {self.state.front_distance}")
                # logger.debug(f"Sonar Side: {self.state.side_distance}")
                # # print("speed: ", self.state.v)
                # # print("steer: ", self.state.steering_angle)

                # # TODO
                # # speed, steer_angle = self.get_controls()

                # # print(f"Time taken {time() - start_time}s\n ========================")
                # # Start car if model if loaded
                # # if not loaded_model.value:
                # #     print("//////////////////// Waiting for Model")
                # #     self.state.v = 0

                # for outP in outPs:
                #     print("Final -> ", (self.state.steering_angle, self.state.v))
                #     outP.send((self.state.steering_angle, self.state.v))

            except Exception as e:
                print("Decision Process error:")
                raise e
            finally:
                pass
                # joblib.dump({"x": rx, "y": ry}, "real_coords.z")
