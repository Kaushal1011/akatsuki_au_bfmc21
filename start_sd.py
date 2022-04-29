import sys
from multiprocessing import Event, Pipe


import argparse

import numpy as np
from src import config as config_module

parser = argparse.ArgumentParser()
parser.add_argument(
    "config_path", help="Path to the config file.", default="./config_pc.json"
)

args = parser.parse_args()
config_module.config_path = args.config_path
config = config_module.get_config()


from src.hardware.camera.CameraSpooferProcess import CameraSpooferProcess
from src.data.localisationssystem.locsysProc import LocalisationSystemProcess
from src.data.server_sim import ServerSIM as LocSysSIM
from src.data.server_sim import ServerSIM as TrafficSIM
from src.data.server_sim import ServerSIM as IMUSIM
from src.data.server_sim import ServerSIM as DistanceSIM

from src.data.localisationssystem.home_locProc import LocalisationProcess
from src.data.trafficlights.trafficProc import TrafficProcess
from src.hardware.camera.cameraprocess import CameraProcess
from src.hardware.camera.SIMCameraProcess import SIMCameraProcess
from src.hardware.serialhandler.SerialHandlerProcess import SerialHandlerProcess
from src.hardware.ultrasonic.distanceProc import DistanceProcess
from src.lib.actuator.momentcontrol import MovementControl
from src.lib.actuator.sim_connect import SimulatorConnector
from src.lib.cortex.decisionproc import DecisionMakingProcess
from src.lib.cortex.posfusproc import PositionFusionProcess
from src.lib.cortex.object_proc import ObjectProcess
from src.lib.perception.intersection_det import IntersectionDetProcess
from src.lib.perception.lanekeep import LaneKeepingProcess as LaneKeeping
from src.lib.perception.signdetection import SignDetectionProcess
from src.utils.camerastreamer.CameraStreamerProcess import CameraStreamerProcess
from src.utils.remotecontrol.RemoteControlReceiverProcess import (
    RemoteControlReceiverProcess,
)
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
# =============================== INITIALIZING PROCESSES =================================
# Pipe collections
allProcesses = []
movementControlR = []
camOutPs = []
dataFusionInputPs = []
dataFusionInputName = []

posFusionInputPs = []
posFusionInputName = []

# Camera process -> Sign Detection
camsDR, camsDS = Pipe(duplex=False)
sDProc = SignDetectionProcess([camsDR], [])

camOutPs.append(camsDS)

allProcesses.append(sDProc)

camProc = CameraProcess([], camOutPs)

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

