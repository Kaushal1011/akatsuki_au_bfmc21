import datetime
import math
from socket import timeout
from threading import Thread
from time import time
from typing import Optional

from src.config import config
from src.lib.cortex.pathplanning import PathPlanning, Purest_Pursuit
from src.templates.workerprocess import WorkerProcess
from time import time
import joblib

import math

class CarState:
    def __init__(self, v=0.12, dt=0.1, car_len=0.365) -> None:
        self.steering_angle = 0.0
        self.det_intersection = False
        # TODO: get initial position from config IDK
        self.x = 0.75
        # 0.75, 4.8
        self.y = 4.8
        self.yaw = 0
        self.tl = {}
        self.v = v
        self.dt = dt
        self.car_len = car_len
        self.rear_x = self.x - ((car_len / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((car_len / 2) * math.sin(self.yaw))
        self.closest_pt = None
        self.last_update_time = time()

    def update_pos(self, steering_angle):
        dt = time() - self.last_update_time
        self.last_update_time = time()
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.car_len * math.tan(steering_angle) * dt

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

    def update(
        self,
        angle: Optional[float] = None,
        det_intersection: Optional[bool] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
        tl: Optional[dict] = None,
    ) -> None:
        self.last_update_time = time()
        if angle:
            self.steering_angle = angle
        if det_intersection:
            self.det_intersection = det_intersection
        if x:
            self.x = x
            self.rear_x = self.x - ((self.car_len / 2) * math.cos(self.yaw))
        if y:
            self.y = y
            self.rear_y = self.y - ((self.car_len / 2) * math.sin(self.yaw))
        if yaw:
            self.yaw = yaw
        if tl:
            self.tl = tl

    def __repr__(self) -> str:
        return f"{datetime.datetime.now()}| {self.steering_angle:.4f}, {self.det_intersection}, {self.x:.4f}, {self.y:.4f}, {self.yaw:.4f}"

    def asdict(self) -> dict:
        return {"angle": self.steering_angle, "intersection": self.det_intersection}


plan = PathPlanning()
a = time()
if config["preplan"] == False:
    coord_list, p_type, etype = plan.get_path(config["start_idx"], config["end_idx"])

else:
    #preplanpath = joblib.load("../nbs/preplan.z")
    #coord_list = [i for i in zip(preplanpath["x"], preplanpath["y"])]
    #coord_list = coord_list[::20][:10]
    #p_type = preplanpath["ptype"]
    #p_type = p_type[::20][:10]
    #etype = preplanpath["etype"]
    #etype = etype[::20][:10]
    x = [0.43333333333333335,
 0.5334228550732018,
 0.6334811521713754,
 0.7334769999861602,
 0.8333791738758624,
 0.9331564491987875,
 1.032777601313242,
 1.1322114055775312,
 1.2314266373499618,
 1.3304098747116546,
 1.429274103042566,
 1.5281870036989258,
 1.6273164767090973,
 1.726830422101445,
 1.826896739904331,
 1.9276833301461198,
 2.029358092855175,
 2.1320820939497263,
 2.2357379701392035,
 2.3398391128611804,
 2.44387346127402,
 2.5473289545360855,
 2.6496935318057404,
 2.7504551322413477,
 2.8491016950012718,
 2.945121159243875,
 3.0380014641275217,
 3.1272305488105734,
 3.2122963164170297,
 3.2923070464833,
 3.3650548846565225,
 3.4280469453401956,
 3.4787903429378173,
 3.514792191852884,
 3.534312602132963,
 3.5393173956817288,
 3.5329104358260794,
 3.518195891054852,
 3.4982779298568856,
 3.4762607207210188,
 3.4552462978494316,
 3.4374758668177914,
 3.4230149170479724,
 3.41159046598552,
 3.4029295310759773,
 3.3967591297648903,
 3.392806279497802,
 3.390797997720257,
 3.3904613018778,
 3.3915232094159737,
 3.3937107377803257,
 3.3967509044163977,
 3.4003707267697347,
 3.404297222285882,
 3.408257408410382,
 3.411963214526098,
 3.4150625714633787,
 3.4171864444311986,
 3.4179657985189165,
 3.4170315988158926,
 3.4140148104114894,
 3.4085463983950657,
 3.4002573278559804,
 3.388778563883595,
 3.373741071567272,
 3.354775815996368,
 3.3315137622602458,
 3.303685248906221,
 3.273995890278028,
 3.24856915348893,
 3.2337169284613294,
 3.2357511051176293,
 3.260920235582299,
 3.311281253947061,
 3.382158472611536,
 3.468268612705182,
 3.5643283953574585,
 3.6650545416978253,
 3.7660242698390323,
 3.866425140109399,
 3.9663914253532617,
 4.0660574020550175,
 4.165557346699065,
 4.265025535769797,
 4.364569473834928,
 4.4641841446717025,
 4.563834975368557,
 4.663487392887649,
 4.763106898964015,
 4.862672149627786,
 4.962189924289969,
 5.0616706246653544,
 5.161124652468726,
 5.260562409414871]
    
    y = [5.375,
  5.383199326445433,
  5.391301011400709,
  5.399207413375672,
  5.406820890880165,
  5.414043802424033,
  5.4207785065171175,
  5.4269273616692635,
  5.4323927263903125,
  5.437096219176442,
  5.4410962130457525,
  5.4445102541982395,
  5.447456125405706,
  5.450051609439958,
  5.452414489072795,
  5.454662547076023,
  5.456913566221444,
  5.4592742546516275,
  5.461400127860006,
  5.46234834187512,
  5.4611348074792465,
  5.456775435454662,
  5.4482861365836435,
  5.434682821648471,
  5.41498140143142,
  5.388197786714767,
  5.3533478882807914,
  5.309447616911768,
  5.255512930556598,
  5.191056689535604,
  5.117314487086862,
  5.0358950036472585,
  4.9484069196536815,
  4.856458915543018,
  4.761504604054688,
  4.664234464423257,
  4.565104614018768,
  4.4645711073679255,
  4.3630899989974345,
  4.261117343434001,
  4.159108897917843,
  4.057400514184635,
  3.956024986735275,
  3.854967964039682,
  3.7542150945677766,
  3.6537520267894728,
  3.55356440917469,
  3.4536378901933467,
  3.353958118315361,
  3.2545107420106496,
  3.155281409749131,
  3.0562557700007233,
  2.9574194712353443,
  2.8587581619229114,
  2.760257490533344,
  2.661892011240951,
  2.5635892199014965,
  2.465264137500859,
  2.3668317849369696,
  2.2682071831077573,
  2.169305352911152,
  2.070041315245083,
  1.9703300910074801,
  1.8700867010962727,
  1.7692261664093905,
  1.6676635078447626,
  1.5653137463003197,
  1.4621151414346454,
  1.3587017294601171,
  1.2565068210040122,
  1.15700778989372,
  1.061682009956629,
  0.9720083822754289,
  0.889566879821516,
  0.816099818314372,
  0.7533641642352502,
  0.7031168840654047,
  0.6671149442860888,
  0.6463421510686829,
  0.6385384022846694,
  0.6405929704617376,
  0.6493951248569669,
  0.6618341347274368,
  0.6747992693302262,
  0.6857932940535565,
  0.6948974624241682,
  0.7028703389901212,
  0.7104704911932761,
  0.7184549430863794,
  0.7273092005566875,
  0.7369382740158863,
  0.7471724058041198,
  0.7578418382615311,
  0.768776813728264]
    coord_list = list(zip(x,y))
    print("Coord",coord_list)
    p_type=["int" for i in range(len(coord_list))]
    etype=[False for i in range(len(coord_list))]
    print("no. of points", len(coord_list))

pPC = Purest_Pursuit(coord_list)
# print("Time taken by Path Planning:", time() - a)


def controlsystem(vehicle: CarState, ind, Lf):
    di = pPC.purest_pursuit_steer_control(vehicle, ind, Lf)
    di = di * 180 / math.pi
    di = di*1.4
    print("DI", di)
    # di = di*0.5
    if di > 21:
        di = 21
    elif di < -21:
        di = -21

    return di


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
        self.state = CarState()
        self.locsys_first = True
        self.inPsnames = inPsnames

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
            args=(self.inPs, self.outPs,),
        )
        thr.daemon = True
        self.threads.append(thr)

    def _the_thread(self, inPs, outPs):
        """Obtains image, applies the required image processing and computes the steering angle value.

        Parameters
        ----------
        inP  : Pipe
            Input pipe to read the frames from other process.
        outP : Pipe
            Output pipe to send the steering angle value to other process.
        """
        use_self_loc = False
        states_l = []
        while True:
            try:
                c = time()

                t_lk = time()
                idx = self.inPsnames.index("lk")
                lk_angle, _ = inPs[idx].recv()
                # print(f"Time taken lk {(time() - t_lk):.3f}s {lk_angle}")

                t_id = time()
                idx = self.inPsnames.index("iD")
                detected_intersection = inPs[idx].recv()
                # print(f"Time taken id {(time() - t_id):.3f}s  {detected_intersection}")
                print(f"Detected {detected_intersection}")

                
                x = self.state.x
                y = self.state.y
                yaw = self.state.yaw
                trafficlights = None
                imu_data = None
                # sign Detection
                t_sD = time()
                if "sD" in self.inPsnames:
                    idx = self.inPsnames.index("sD")
                    if inPs[idx].poll(timeout=0.1):
                        label, area = inPs[idx].recv()
                        print(f"Time taken sD {(time() - t_sD):.2f}s {label}")

                # locsys
                t_loc = time()
                if "loc" in self.inPsnames:
                    idx = self.inPsnames.index("loc")
                    if self.locsys_first:
                        if inPs[idx].poll(timeout=0.3):
                            loc = inPs[idx].recv()

                            x = loc["posA"]
                            y = loc["posB"]
                            if "rotA" in loc.keys():
                                yaw = 2 * math.pi - (loc["rotA"] + math.pi)
                            elif "radA" in loc.keys():
                                yaw = 2 * math.pi - (loc["radA"] + math.pi)

                            self.locsys_first = False
                        use_self_loc = False

                    if inPs[idx].poll(timeout=0.1):
                        loc = inPs[idx].recv()
                        use_self_loc = False
                        x = loc["posA"]
                        y = loc["posB"]
                        if "rotA" in loc.keys():
                            yaw = 2 * math.pi - (loc["rotA"] + math.pi)
                        elif "radA" in loc.keys():
                            yaw = 2 * math.pi - (loc["radA"] + math.pi)
                    else:
                        use_self_loc = True
                    # print(f"Time taken loc {(time() - t_loc):.2f}s {loc}")

                # TODO: add back
                # # if trafficlight process is connected
                # if len(inPs) > 3:
                #     trafficlights = inPs[3].recv()
                #     print(trafficlights)

                # imu
                if "imu" in self.inPsnames:
                    idx = self.inPsnames.index("imu")
                    if inPs[idx].poll(timeout=0.1):
                        imu_data = inPs[idx].recv()
                        yaw_imu = imu_data["yaw"]
                        # print("IMU:", imu_data)
                        # yaw_imu = 360 - imu_data["yaw"]
                        # print("imu_yaw", yaw_imu, "yaw", yaw)
                        #yaw_imu = yaw_imu if yaw_imu > 180 else -yaw_imu
                        yaw_imu = math.radians(yaw_imu)
                        if yaw_imu > math.pi:
                            yaw_imu -= 2*math.pi
                        yaw_f = yaw_imu
                        print("yaw", yaw_f)

                self.state.update(
                    lk_angle, detected_intersection, x, y, yaw_f, trafficlights
                )
            
                # print(self.state)
                ind, Lf = pPC.search_target_index(self.state)
                # print("searched")
                print(f"({x}, {y}) -> {ind}, {coord_list[ind]}")
                # if p_type[ind - 1] == "int":
                angle = controlsystem(self.state, ind, Lf)
                print(f"Angle {angle}")
                
                states_l.append((x, y, yaw_f,coord_list[ind], angle))
                print(states_l)
                # lane keeping
                #elif p_type[ind - 1] == "lk":
                #    angle = lk_angle
                #    angle2 = controlsystem(self.state, ind, Lf)
                #    if abs(angle2 - angle) > 21:
                #        print("lane keeping failed")
                #        angle = angle2
                #else:
                #    print("Here in nothingness")

                # print(f"Current Behaviour : {p_type[ind-1]}")

                # if no locsys use self localization
                if len(inPs) < 3 or use_self_loc:
                    #print("Using self localization")
                    #self.state.update_pos(angle)
                    pass

                for outP in outPs:
                    outP.send((angle, None))
                print(time() - c)
                
            except Exception as e:
                print("Decision Process error:")
                raise e
                