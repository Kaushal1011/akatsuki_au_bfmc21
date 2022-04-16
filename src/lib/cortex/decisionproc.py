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
from src.templates.workerprocess import WorkerProcess
from time import time
from src.lib.cortex.action import ActionBehaviour,ActionManager,LaneKeepBehaviour,ControlSystemBehaviour, ObjectStopBehaviour,StopBehvaiour,PriorityBehaviour
import joblib

import math


def get_last(inP: Pipe, delta_time: float = 1e-2):
    timestamp, data = inP.recv()

    while (time() - timestamp) > delta_time:
        timestamp, data = inP.recv()
    return timestamp, data


def trigger_behaviour(carstate: CarState,action_man:ActionManager):
    # print(carstate.detected_sign)
    if carstate.detected_intersection and carstate.current_ptype == "int":
        # intersection
        pass
    
    if carstate.detected_intersection :
        pass
        # stop for t secs intersection
        stopobj=StopBehvaiour()
        stopaction=ActionBehaviour(name="stop",release_time=6.0,callback=stopobj)
        action_man.set_action(stopaction,action_time=3.0)


    if carstate.detected_sign["parking"]:
        # Parking
        pass

    if carstate.detected_car and carstate.can_overtake:
        # overtake
        pass

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
        stopobj=StopBehvaiour()
        stopaction=ActionBehaviour(name="stop",release_time=6.0,callback=stopobj)
        action_man.set_action(stopaction,action_time=3.0)

    if carstate.detected_sign["priority"]:
        # slowdown for t secs
        pass

    if carstate.detected_pedestrian or carstate.detected_sign["crosswalk"]:
        # stop detected pedestrain or crosswalk
        pass

    if carstate.pitch > 0.2:
        # incline
        pass

    if carstate.pitch < -0.2:
        # decline
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

        self.actman=ActionManager()

        lkobj=LaneKeepBehaviour()
        lkaction=ActionBehaviour(name="lk",callback=lkobj)
        self.actman.set_action(lkaction)
        data_path = pathlib.Path(pathlib.Path(__file__).parent.parent.parent.resolve(), "data", "mid_course.z")
        data=joblib.load(data_path)
        # pass coordlist here from navigator config 
        csobj=ControlSystemBehaviour(coord_list=data[0])
        csaction=ActionBehaviour(name="cs",callback=csobj)
        self.actman.set_action(csaction)

        # pass coordlist here from navigator config 
        stopobj=ObjectStopBehaviour()
        stopobjaction=ActionBehaviour(name="objstop",callback=stopobj)
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

    def _the_thread(self, inPs:List[Connection], outPs:List[Connection]):
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
                lk_angle, _ = inPs[idx].recv()
                self.state.update_lk_angle(lk_angle)
                print("lk")
                # print(f"Time taken lk {(time() - t_lk):.4f}s {lk_angle}")

                t_id = time()
                idx = self.inPsnames.index("iD")
                detected_intersection = inPs[idx].recv()
                self.state.update_intersection(detected_intersection)
                print("id: ",detected_intersection)
                # print(f"TIme taken iD {(time()- t_id):.4f}s")

                # sign Detection
                t_sD = time()
                if "sD" in self.inPsnames:
                    idx = self.inPsnames.index("sD")
                    if inPs[idx].poll():
                        # TODO : get sign detection for all signs
                        sign, sign_area = inPs[idx].recv()
                        # self.state.update_sign_detected()
                        print("sd")

                if "pos" in self.inPsnames:
                    idx = self.inPsnames.index("pos")
                    if inPs[idx].poll():
                        pos=inPs[idx].recv()
                        # print("pos")
                        # print("Position: ",pos)
                        if pos[0]==0 and pos[1]==0:
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

                trigger_behaviour(self.state,self.actman)

                speed,steer=self.actman(self.state)
                self.state.v=speed
                self.state.steering_angle=steer

                # print("speed: ", self.state.v)
                # print("steer: ", self.state.steering_angle)

                # TODO
                # speed, steer_angle = self.get_controls()

                # print(f"Time taken {time() - start_time}s\n ========================")
                for outP in outPs:
                    outP.send((self.state.steering_angle, self.state.v))

            except Exception as e:
                print("Decision Process error:")
                raise e
