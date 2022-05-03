import sys
from multiprocessing import Event, Pipe


import argparse

from isort import stream
from src import config as config_module

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

from src.data.server_sim import ServerSIM as DistanceSIM
from src.hardware.ultrasonic.distanceProc import DistanceProcess
from src.lib.actuator.momentcontrol import MovementControl
from src.lib.actuator.sim_connect import SimulatorConnector
from src.hardware.serialhandler.SerialHandlerProcess import SerialHandlerProcess

from src.utils.camerastreamer.zmqStreamerProcess import CameraStreamerProcess
from src.utils.remotecontrol.RemoteControlReceiverProcess import (
    RemoteControlReceiverProcess,
)
from src.lib.cortex.decisionproc import DecisionMakingProcess

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
    logger.add(sys.stderr, filter=filter([18]))

logger.add("file1.log", filter=lambda r: r["level"] == 14)
# logger.level("LK", no=10, color="<blue>", icon='' )
# logger.level("INT", no=10, color="<blue>", icon='' )


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
allProcesses = []
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

lkProc = LaneKeeping([], [], enable_stream=("lk" in streams))
allProcesses.append(lkProc)
dataFusionInputName.append("lk")
camOutNames.append("lk")

if not config["enableSignDet"]:
    if "sd" in streams:
        streams.remove("sd")

if config["enableSignDet"]:
    sDProc = SignDetectionProcess([], [], [], enable_stream=("sd" in streams))
    allProcesses.append(sDProc)
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


elif config["using_server"]:
    # LocSys -> Position Fusion
    locsysProc = LocalisationSystemProcess([], [])
    allProcesses.append(locsysProc)
    posFusionInputName.append("loc")


# -------TrafficLightSemaphore----------
# if config["enableSIM"]:
#     # Traffic Semaphore -> Decision Making (data fusion)
#     tlFzzR, tlFzzS = Pipe(duplex=False)
#     trafficProc = TrafficSIM([], [tlFzzS], TRAFFIC_SIM_PORT)
#     allProcesses.append(trafficProc)
#     dataFusionInputPs.append(tlFzzR)
#     dataFusionInputName.append("tl")

# elif config["using_server"]:
#     # Traffic Semaphore -> Decision Making (data fusion)
#     tlFzzR, tlFzzS = Pipe(duplex=False)
#     trafficProc = TrafficProcess([], [tlFzzS])
#     allProcesses.append(trafficProc)
#     dataFusionInputPs.append(tlFzzR)
#     dataFusionInputName.append("tl")

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


#
# # ===================== Position Fusion ==========================================
if len(posFusionInputName) > 0:
    posfzzProc = PositionFusionProcess([], [], inPsnames=posFusionInputName)
    allProcesses.append(posfzzProc)
    dataFusionInputName.append("pos")

# ===================== Distance Sensor ==========================================
# Distance Sensor -> Decision Making (data fusion)

if config["enableSIM"]:
    disProc = DistanceSIM([], [], "dis", 6666)
    allProcesses.append(disProc)
    dataFusionInputName.append("dis")

elif isPI:
    disProc = DistanceProcess([], [])
    allProcesses.append(disProc)
    dataFusionInputName.append("dis")


# ======================= Decision Making =========================================
datafzzProc = DecisionMakingProcess([], [], inPsnames=dataFusionInputName)
allProcesses.append(datafzzProc)
# movementControlR.append(FzzMcR)
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
        camProc = SIMCameraProcess([], [])
    else:
        camProc = CameraProcess([], [], camOutNames)

    allProcesses.append(camProc)


# ===================================== START PROCESSES ==================================
print("Starting the processes!", allProcesses)
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
