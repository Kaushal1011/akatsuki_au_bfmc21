import sys
from multiprocessing import Event, Pipe
from src.hardware.camera.cameraprocess import CameraProcess
from src.lib.perception.signdetection import SignDetectionProcess
import sys

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
# Sign Detection -> Stream
sDStR, sDStS = Pipe(duplex=False)

sDProc = SignDetectionProcess([camsDR], [sDStS], outPnames=["stream"])

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
