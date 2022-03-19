# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
from multiprocessing import Event, Pipe

from src.config import config
from src.data.localisationssystem.locsysProc import LocalisationSystemProcess
from src.data.server_sim import ServerSIM as LocSysSIM
from src.data.server_sim import ServerSIM as TrafficSIM
from src.data.localisationssystem.home_locProc import LocalisationProcess
from src.data.trafficlights.trafficProc import TrafficProcess
from src.hardware.camera.cameraprocess import CameraProcess
from src.hardware.camera.CameraSpooferProcess import CameraSpooferProcess
from src.hardware.camera.SIMCameraProcess import SIMCameraProcess
from src.hardware.serialhandler.SerialHandlerProcess import SerialHandlerProcess
from src.lib.actuator.momentcontrol import MovementControl
from src.lib.actuator.sim_connect import SimulatorConnector
from src.lib.cortex.decisionproc import DecisionMakingProcess
from src.lib.perception.intersection_det import IntersectionDetProcess
from src.lib.perception.lanekeep import LaneKeepingProcess as LaneKeeping
from src.lib.perception.signdetection import SignDetectionProcess
from src.utils.camerastreamer.CameraStreamerProcess import CameraStreamerProcess
from src.utils.remotecontrol.RemoteControlReceiverProcess import (
    RemoteControlReceiverProcess,
)

isPI = True
try:
    from src.utils.IMU.imuProc import IMUProcess
except Exception as e:
    print(e)
    isPI = False
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
# Note: ensure the sequence in which pipes are added to dataFusionInputPs
# [LaneKeep, IntersectionDet, SignDetection, LocSys, TrafficLight]
dataFusionInputPs = []
dataFusionInputName = []
# =============================== RC CONTROL =================================================
if config["enableRc"]:
    # rc      ->  serial handler
    rcShR, rcShS = Pipe(duplex=False)

    rcProc = RemoteControlReceiverProcess([], [rcShS])
    allProcesses.append(rcProc)


# ===================================== PERCEPTION ===================================

if config["enableLaneKeeping"]:
    # Camera process -> Lane keeping
    lkR, lkS = Pipe(duplex=False)

    # Lane keeping -> Data Fusion
    lkFzzR, lkFzzS = Pipe(duplex=False)

    # Lane keeping -> Movement control
    FzzMcR, FzzMcS = Pipe(duplex=False)

    camOutPs.append(lkS)
    dataFusionInputPs.append(lkFzzR)
    dataFusionInputName.append("lk")

    if config["enableStream"]:
        lkStrR, lkStrS = Pipe(duplex=False)
        lkProc = LaneKeeping([lkR], [lkFzzS, lkStrS])
    else:
        lkProc = LaneKeeping([lkR], [lkFzzS])

    allProcesses.append(lkProc)

if config["enableIntersectionDet"]:
    # Camera process -> Intersection Detection
    camiDR, camiDS = Pipe(duplex=False)

    # Intersection Detection -> Data Fusion
    iDFzzR, iDFzzS = Pipe(duplex=False)

    camOutPs.append(camiDS)
    dataFusionInputPs.append(iDFzzR)
    dataFusionInputName.append("iD")

    if config["enableStream"]:
        idStrR, idStrS = Pipe(duplex=False)
        idProc = IntersectionDetProcess([camiDR], [iDFzzS, idStrS])
    else:
        idProc = IntersectionDetProcess([camiDR], [iDFzzS])
    allProcesses.append(idProc)

if config["enableSignDet"]:
    # Camera process -> Sign Detection
    camsDR, camsDS = Pipe(duplex=False)

    # Sign Detection -> Data Fusion (Decision Process)
    sDFzzR, sDFzzS = Pipe(duplex=False)
    if config["enableStream"]:
        sDStR, sDStS = Pipe(duplex = False)
        sDProc = SignDetectionProcess([camsDR], [sDFzzS, sDStS])
    else:
        sDProc = SignDetectionProcess([camsDR], [sDFzzS])
    
    camOutPs.append(camsDS)
    dataFusionInputPs.append(sDFzzR)
    dataFusionInputName.append("sD")

    # TODO: To Streamer / Dashboard
    allProcesses.append(sDProc)

# =============================== DATA ===================================================

# -------LOCSYS----------
if config["enableSIM"]:
    # LocSys -> Decision Making (data fusion)
    lsFzzR, lsFzzS = Pipe(duplex=False)
    locsysProc = LocSysSIM([], [lsFzzS], LOCSYS_SIM_PORT)
    allProcesses.append(locsysProc)
    dataFusionInputPs.append(lsFzzR)
    dataFusionInputName.append("loc")

elif config["home_loc"]:
    # LocSys -> Decision Making (data fusion)
    print("Starting Home Loc Sys")
    lsFzzR, lsFzzS = Pipe(duplex=False)
    locsysProc = LocalisationProcess([], [lsFzzS])
    allProcesses.append(locsysProc)
    dataFusionInputPs.append(lsFzzR)
    dataFusionInputName.append("loc")


elif config["using_server"]:
    # LocSys -> Decision Making (data fusion)
    lsFzzR, lsFzzS = Pipe(duplex=False)
    locsysProc = LocalisationSystemProcess([], [lsFzzS])
    allProcesses.append(locsysProc)
    dataFusionInputPs.append(lsFzzR)
    dataFusionInputName.append("loc")


# -------TrafficLightSemaphore----------
# TODO: enable again when required
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

# -------IMU----------
# IMU -> Decision Making (data fusion)
if isPI and not config["enableSIM"]:
    print("IMU process started")
    imuFzzR, imuFzzS = Pipe(duplex=False)
    imuProc = IMUProcess([], [imuFzzS])
    allProcesses.append(imuProc)
    dataFusionInputPs.append(imuFzzR)
    dataFusionInputName.append("imu")


# ======================= Decision Making =========================================

datafzzProc = DecisionMakingProcess(
    dataFusionInputPs, [FzzMcS], inPsnames=dataFusionInputName
)
allProcesses.append(datafzzProc)
movementControlR.append(FzzMcR)


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
    shProc = SimulatorConnector([mcSSHR], [])
    allProcesses.append(shProc)
    shProc = SerialHandlerProcess([ mcSHR], [])
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
if config["enableStream"]:
    # if config["enableLaneKeeping"] and config["enableIntersectionDet"]:
    #     # shouldnt idstr go here also ?
    #     streamProc = CameraStreamerProcess([lkStrR], [])
    if config["enableLaneKeeping"]:
        streamProc = CameraStreamerProcess([lkStrR], [], STREAM_PORT1)
        allProcesses.append(streamProc)
    else:
        camStR, camStS = Pipe(duplex=False)  # camera  ->  streamer
        camOutPs.append(camStS)
        streamProc = CameraStreamerProcess([camStR], [], STREAM_PORT1)
        allProcesses.append(streamProc)

    if config["enableSignDet"]:
        streamProc2 = CameraStreamerProcess([sDStR], [], STREAM_PORT2)
        allProcesses.append(streamProc2)
# ========================== Camera process ==============================================
if config["enableCameraSpoof"]:
    camSpoofer = CameraSpooferProcess([], camOutPs, "vid")
    allProcesses.append(camSpoofer)
else:
    if config["enableSIM"]:
        camProc = SIMCameraProcess([], camOutPs)
    else:
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
