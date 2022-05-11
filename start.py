from multiprocessing import Process
import sys
from multiprocessing import Event, Pipe


import argparse

from multiprocessing.connection import Connection

from src import config as config_module
from time import sleep

parser = argparse.ArgumentParser()
parser.add_argument(
    "config_path", help="Path to the config file.", default="./config_pc.json"
)

args = parser.parse_args()
config_module.config_path = args.config_path
config = config_module.get_config()


from src.hardware.camera.CameraSpooferProcess import CameraSpooferProcess
from src.hardware.camera.cameraprocess import CameraProcess
from src.hardware.camera.SIMCameraProcess import SIMCameraProcess
from src.lib.perception.lanekeepz import LaneKeepingProcess as LaneKeeping
from src.lib.perception.signdetection import SignDetectionProcess
from src.data.localisationssystem.home_locProc import LocalisationProcess
from src.data.localisationssystem.locsysProc import LocalisationSystemProcess
from src.data.server_sim import ServerSIM as LocSysSIM
from src.data.server_sim import ServerSIM as IMUSIM
from src.lib.cortex.posfusproc import PositionFusionProcess
from src.data.environmentalserver.environmental import EnvironmentalHandler
from src.data.distance_sim import DistanceSIM
from src.hardware.ultrasonic.distanceProc import DistanceProcess
from src.data.trafficlights.trafficProc import TrafficProcess
from src.data.server_sim import ServerSIM as TrafficSIM
from src.lib.actuator.momentcontrol import MovementControl
from src.lib.actuator.sim_connect import SimulatorConnector
from src.hardware.serialhandler.SerialHandlerProcess import SerialHandlerProcess

from src.utils.camerastreamer.zmqStreamerProcess import CameraStreamerProcess
from src.utils.remotecontrol.RemoteControlReceiverProcess import (
    RemoteControlReceiverProcess,
)
from src.lib.cortex.decisionproc import DecisionMakingProcess
from src.lib.perception.signdetection import loaded_model

import sys
from loguru import logger
from typing import List

isPI = True
try:
    from src.utils.IMU.imuProc import IMUProcess
except Exception as e:
    print(e)
    isPI = False
# =================== CONFIG LOGGER ======================================

logger.level("PIPE", no=12, icon="==")
logger.level("SYNC", no=13, color="<yellow>")
logger.level("XY", no=14)
logger.level("TIME", no=15)


def filter(level: List[int]):
    return lambda r: r["level"].no in level or r["level"].no > 19


TEST_PIPE = True
logger.remove()
if TEST_PIPE:
    logger.add(
        sys.stderr,
        filter=filter([18]),
        format="<level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
    )

logger.add(
    "file1.log",
    filter=lambda r: r["level"] == 14,
    format="<level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
)


# ========================================================================
# SCRIPT USED FOR WIRING ALL COMPONENTS
# ========================================================================
sys.path.append(".")

TRAFFIC_SIM_PORT = 7777
LOCSYS_SIM_PORT = 8888
LOCSYS_HOME_PORT = 8888

STREAM_PORT1 = 2244
STREAM_PORT2 = 4422
# ["cam", "lk", "sd"]

streams = ["sd"]
# =============================== INITIALIZING PROCESSES =================================
# Pipe collections
allProcesses: List[Process] = []
movementControlR = []
dataFusionInputName = []
posFusionInputName = []
camOutNames = []
# =============================== RC CONTROL =================================================
if config["enableRc"]:
    # rc  ->  serial handler
    rcShR, rcShS = Pipe(duplex=False)
    rcProc = RemoteControlReceiverProcess([], [rcShS])
    allProcesses.append(rcProc)


# ===================================== PERCEPTION ===================================
dataFusionOutPs: List[Connection] = []

sDProc = None
if config["enableLaneKeeping"]:
    lkProc = LaneKeeping([], [], enable_stream=("lk" in streams))
    allProcesses.append(lkProc)
    dataFusionInputName.append("lk")
    camOutNames.append("lk")

if not config["enableSignDet"]:
    if "sd" in streams:
        streams.remove("sd")

if config["enableSignDet"]:
    print("Sign Detection will start in a while")
    sDProc = SignDetectionProcess([], [], [], enable_stream=("sd" in streams))
    # allProcesses.append(sDProc)
    dataFusionInputName.append("sd")
    camOutNames.append("sd")


# =============================== DATA ===================================================

# -------LOCSYS----------
if config["enableSIM"]:
    # LocSys -> Position Fusion
    locsysProc = LocSysSIM([], [], "loc", LOCSYS_SIM_PORT)
    allProcesses.append(locsysProc)
    posFusionInputName.append("loc")

elif config["home_loc"]:
    # LocSys -> Position Fusion
    print(">>> Starting Home Localization process")
    locsysProc = LocalisationProcess([], [])
    allProcesses.append(locsysProc)
    posFusionInputName.append("loc")


elif config["loc_server"]:
    # LocSys -> Position Fusion
    locsysProc = LocalisationSystemProcess([], [])
    allProcesses.append(locsysProc)
    posFusionInputName.append("loc")


# -------TrafficLightSemaphore----------
if config["enableSIM"]:
    # Traffic Semaphore -> Decision Making (data fusion)
    trafficProc = TrafficSIM([], [], "tl", TRAFFIC_SIM_PORT)
    allProcesses.append(trafficProc)
    dataFusionInputName.append("tl")

elif config["tl_server"]:
    # Traffic Semaphore -> Decision Making (data fusion)
    trafficProc = TrafficProcess([], [])
    allProcesses.append(trafficProc)
    dataFusionInputName.append("tl")


#
# ===================== Distance Sensor ==========================================
# Distance Sensor -> Decision Making (data fusion)

if config["enableSIM"]:
    disProc = DistanceSIM([], [], 6666, log=False)
    allProcesses.append(disProc)
    dataFusionInputName.append("dis")

elif isPI:
    disProc = DistanceProcess([], [])
    allProcesses.append(disProc)
    dataFusionInputName.append("dis")

# ========================= IMU ===================================================
# IMU -> Position Fusino
if isPI and not config["enableSIM"]:
    print("IMU process started")
    imuProc = IMUProcess([], [])
    allProcesses.append(imuProc)
    posFusionInputName.append("imu")

else:
    imuProc = IMUSIM([], [], "imu", 5555)
    allProcesses.append(imuProc)
    posFusionInputName.append("imu")


# # ===================== Position Fusion ==========================================
if len(posFusionInputName) > 0:
    posfzzProc = PositionFusionProcess([], [], inPsnames=posFusionInputName)
    allProcesses.append(posfzzProc)
    dataFusionInputName.append("pos")


# ==== Movement Control pipe
# Decision Process -> Movement control
FzzMcR, FzzMcS = Pipe(duplex=False)
dataFusionOutPs.append(FzzMcS)

# ======================= Environment Server ======================================
if config["env_server"]:
    beacon = 23456
    id = 120
    serverpublickey = "publickey_server_test.pem"
    clientprivatekey = "privatekey_client_test.pem"

    gpsStR, gpsStS = Pipe(duplex=False)

    envhandler = EnvironmentalHandler(
        id, beacon, serverpublickey, gpsStR, clientprivatekey
    )
    allProcesses.append(envhandler)
    dataFusionOutPs.append(gpsStS)


# ======================= Decision Making =========================================
# TODO enableTelemtry
if False:
    dataFusionInputName.append("tel")

datafzzProc = DecisionMakingProcess([], dataFusionOutPs, inPsnames=dataFusionInputName)
allProcesses.append(datafzzProc)
movementControlR.append(FzzMcR)
#

# ======================= Actuator =================================================

# Movement control
if config["enableSIM"] and isPI:
    # Movement control -> Serial handler
    mcSHR, mcSHS = Pipe(duplex=False)
    # Moment control -> SIM Serial Handler
    mcSSHR, mcSSHS = Pipe(duplex=False)
    cfProc = MovementControl(movementControlR, [mcSHS, mcSSHS])
    allProcesses.append(cfProc)
else:
    # Movement control -> Serial handler
    mcSHR, mcSHS = Pipe(duplex=False)
    cfProc = MovementControl(movementControlR, [mcSHS])
    allProcesses.append(cfProc)

# Serial handler or Simulator Connector
if config["enableSIM"] and isPI:
    # shProc = SimulatorConnector([mcSSHR], [])
    # allProcesses.append(shProc)

    shProc = SerialHandlerProcess([mcSHR], [])
    allProcesses.append(shProc)

elif config["enableSIM"] and not isPI:
    shProc = SimulatorConnector([mcSHR], [])
    allProcesses.append(shProc)
else:
    try:
        shProc = SerialHandlerProcess([mcSHR], [])
        allProcesses.append(shProc)
    except Exception:
        print("ERROR: Falied to start Serial Handler")


# ========================= Streamer =====================================================
if "cam" in streams:
    camOutNames.append("stream")

if config["enableStream"]:
    streamProc = CameraStreamerProcess([], [], streams[0], port=STREAM_PORT1)
    allProcesses.append(streamProc)

    if len(streams) > 1:
        streamProc2 = CameraStreamerProcess([], [], streams[1], port=STREAM_PORT2)
        allProcesses.append(streamProc2)


# ========================== Camera process ==============================================
if config["enableCameraSpoof"]:
    camSpoofer = CameraSpooferProcess([], [], "vid")
    allProcesses.append(camSpoofer)
else:
    if config["enableSIM"]:
        camProc = SIMCameraProcess([], [], camOutNames)
    else:
        camProc = CameraProcess([], [], camOutNames)

    allProcesses.append(camProc)


# ===================================== START PROCESSES ==================================
print("Starting the processes!", allProcesses)
if sDProc is not None:
    sDProc.daemon = True
    sDProc.start()
    while not loaded_model.value:
        print("Waiting on sDProc")
        sleep(1)
        
    for proc in allProcesses:
        proc.daemon = True
        proc.start()
    allProcesses.append(sDProc)
else:
    for proc in allProcesses:
        proc.daemon = True
        proc.start()    


# ===================================== STAYING ALIVE ====================================
blocker = Event()

try:
    blocker.wait()
except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
    for proc in allProcesses:
        if hasattr(proc, "stop") and callable(getattr(proc, "stop")):
            print("Process with stop", proc)
            proc.stop()
            proc.join()
        else:
            print("Process witouth stop", proc)
            proc.terminate()
            proc.join()
