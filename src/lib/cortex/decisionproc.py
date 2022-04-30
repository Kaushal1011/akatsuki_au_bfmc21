import datetime
import math
from multiprocessing import Pipe
from multiprocessing.connection import Connection
import pathlib
from threading import Thread
from time import time
from typing import List, Optional
from src.lib.cortex.pathplanning import (
    PathPlanning,
    Purest_Pursuit,
    give_perpendicular_park_pts,
)

from src.lib.cortex.carstate import CarState
from src.lib.perception.signdetection import loaded_model
from src.templates.workerprocess import WorkerProcess
from time import time
from src.lib.cortex.action import (
    ActionBehaviour,
    ActionManager,
    LaneKeepBehaviour,
    ControlSystemBehaviour,
    ObjectStopBehaviour,
    StopBehvaiour,
    PriorityBehaviour,
    OvertakeBehaviour,
)
import joblib
from loguru import logger
import math

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

    if carstate.detected_sign["parking"]:
        # Parking
        pass

    # if carstate.detected_car and carstate.can_overtake:
    #     # overtake
    #     pass

    if carstate.front_distance < 0.5:
        # print("Overtake Trigger")
        # overtake
        overtakeobj = OvertakeBehaviour(car_state=carstate)
        overtakeobjaction = ActionBehaviour(name="overtaking", callback=overtakeobj)
        action_man.set_action(overtakeobjaction, action_time=None, car_state=carstate)

    if carstate.detected_car and not carstate.can_overtake:
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
        pass

    if carstate.detected_pedestrian or carstate.detected_sign["crosswalk"]:
        # stop detected pedestrain or crosswalk
        pass

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
        while True:
            try:
                c = time()
                start_time = time()
                t_lk = time()
                idx = self.inPsnames.index("lk")
                lk_timestamp, lk_angle, detected_intersection = get_last_lk_id(
                    inPs[idx]
                )
                self.state.update_lk_angle(lk_angle)
                self.state.update_intersection(detected_intersection)
                logger.log("PIPE", f"Recv->LK {lk_angle}")
                logger.log("SYNC", f"LK timedelta {time()- lk_timestamp}")
                # print(f"Time taken lk {(time() - t_lk):.4f}s {lk_angle}")

                # t_id = time()
                # idx = self.inPsnames.index("iD")
                # id_timestamp, detected_intersection = get_last_value(inPs[idx])
                # logger.log("PIPE", f"Recv->ID {detected_intersection}")
                # logger.log("SYNC", f"iD delta {time()- id_timestamp}")

                # print("id: ", detected_intersection)
                # print(f"TIme taken iD {(time()- t_id):.4f}s")

                # sign Detection
                # TODO
                t_sD = time()
                if "sD" in self.inPsnames:
                    idx = self.inPsnames.index("sD")
                    if inPs[idx].poll():
                        # TODO : get sign detection for all signs
                        sd_timestamp, sign = inPs[idx].recv()
                        print("SD <-<", sign)
                        logger.log("SYNC", f"SD timedelta {time() - sd_timestamp}")
                        logger.log("PIPE", f"Recv -> SD {sign}")

                        # self.state.update_sign_detected()
                # TODO
                if "obj" in self.inPsnames:
                    if inPs[idx].poll(timeout=0.01):
                        idx = self.inPsnames.index("obj")
                        obj_data = inPs[idx].recv()
                        self.state.update_object_det(*obj_data)
                        logger.log("PIPE", f"Recv->OBJ {obj_data}")

                if "dis" in self.inPsnames:
                    idx = self.inPsnames.index("dis")
                    distance_data = get_last_distance(inPs[idx])
                    final_data = (
                        distance_data["sonar1"],
                        distance_data["sonar2"],
                        False,
                        False,
                        False,
                    )
                    # print("distance", distance_data["sonar1"], distance_data["sonar1"])
                    logger.log("PIPE", f"Recv->DIS {final_data[0]},{final_data[1]}")
                    logger.log(
                        "SYNC", f"dis delta {time()- distance_data['timestamp']}"
                    )

                    self.state.update_object_det(*final_data)

                if "pos" in self.inPsnames:
                    idx = self.inPsnames.index("pos")
                    if inPs[idx].poll():
                        pos_timestamp, pos = get_last_value(inPs[idx])
                        logger.log(
                            "PIPE",
                            f"Recv->POS {pos[0]:.2f} {pos[1]:.2f} {pos[2]:.2f} {pos[3]:.2f} {pos[4]:.2f}",
                        )
                        logger.log("SYNC", f"pos delta {time()- pos_timestamp}")
                        print(f"({pos[0]:.3f}, {pos[1]:.3f}) YAW {pos[2]:.3f}")
                        # print("pos")
                        # print("Position: ",pos)
                        if pos[0] == 0 and pos[1] == 0:
                            pass
                        else:
                            self.state.update_pos(*pos)
                    else:
                        self.state.update_pos_noloc()

                # TODO: add back
                # # if trafficlight process is connected
                # if len(inPs) > 3:
                #     trafficlights = inPs[3].recv()
                #     print(trafficlights)

                # update car navigator, current ptype, current etype and current idx

                trigger_behaviour(self.state, self.actman)

                speed, steer = self.actman(self.state)
                self.state.v = speed
                self.state.steering_angle = steer
                rx.append(self.state.x)
                ry.append(self.state.y)
                logger.log("XY", f"{self.state.x}, {self.state.y},")
                logger.debug(f"Sonar Front: {self.state.front_distance}")
                logger.debug(f"Sonar Side: {self.state.side_distance}")
                # print("speed: ", self.state.v)
                # print("steer: ", self.state.steering_angle)

                # TODO
                # speed, steer_angle = self.get_controls()

                # print(f"Time taken {time() - start_time}s\n ========================")
                # Start car if model if loaded
                if not loaded_model.value:
                    self.state.v = 0

                for outP in outPs:
                    print("Final -> ", (self.state.steering_angle, self.state.v))
                    outP.send((self.state.steering_angle, self.state.v))

            except Exception as e:
                print("Decision Process error:")
                raise e
            finally:
                joblib.dump({"x": rx, "y": ry}, "real_coords.z")
