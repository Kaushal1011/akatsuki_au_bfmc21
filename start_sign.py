import sys
from multiprocessing import Event


import argparse

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
from src.lib.perception.signdetection import SignDetectionProcess

from src.utils.camerastreamer.zmqStreamerProcess import CameraStreamerProcess

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
logger.level("SD",no=16 )

def filter(level: List[int]):
    return lambda r: r["level"].no in level or r["level"].no > 19


TEST_PIPE = True
logger.remove()
if TEST_PIPE:
    logger.add(sys.stderr, filter=filter([16]))

# ========================================================================
# SCRIPT USED FOR WIRING ALL COMPONENTS
# ========================================================================
sys.path.append(".")

STREAM_PORT1 = 2244
STREAM_PORT2 = 4422
# ["cam", "lk", "sd"]

streams = ["cam"]
# =============================== INITIALIZING PROCESSES =================================
# Pipe collections
allProcesses = []
movementControlR = []
dataFusionInputName = []
posFusionInputName = []
camOutNames = []
# ===================================== PERCEPTION ===================================

if not config["enableSignDet"]:
    if "sd" in streams:
        streams.remove("sd")

if config["enableSignDet"]:
    sDProc = SignDetectionProcess([], [], [], enable_stream=("sd" in streams))
    allProcesses.append(sDProc)
    dataFusionInputName.append("sd")
    camOutNames.append("sd")

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
