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

# ========================================================================
# SCRIPT USED FOR WIRING ALL COMPONENTS
# ========================================================================
from fastapi import Path
from src.utils.controlsys.momentcontrol import MovementControl
from src.utils.controlsys.lanekeeping import LaneKeepingProcess as LaneKeeping
from src.utils.remotecontrol.RemoteControlReceiverProcess import (
    RemoteControlReceiverProcess,
)

# from src.utils.IMU.imuProc import IMUProcess
from src.utils.controlsys.intersection_det import IntersectionDetProcess
from src.utils.decisionproc import DecisionMakingProcess as DataFusionProcess
from src.utils.camerastreamer.CameraStreamerProcess import CameraStreamerProcess
from src.hardware.camera.SIMCameraProcess import SIMCameraProcess
from src.hardware.serialhandler.SerialHandlerProcess import SerialHandlerProcess
from src.utils.controlsys.father import SimulatorConnector
from src.data.localisationssystem.localisation4sim import LocSysSIM
from src.data.trafficlights.trafficSIMProc import TrafficSIM
from src.hardware.camera.CameraSpooferProcess import CameraSpooferProcess
from src.hardware.camera.cameraprocess import CameraProcess
from src.utils.pathplanning import PathPlanningProcess
from multiprocessing import Pipe, Process, Event
import sys

sys.path.append(".")


# hardware imports

# utility imports

# =============================== CONFIG =================================================
enableStream = False
enableCameraSpoof = False
enableRc = False
enableLaneKeeping = True
enableSIM = True
enableIntersectionDet = True

# =============================== INITIALIZING PROCESSES =================================
# Pipe collections
allProcesses = list()
movementControlR = list()
camOutPs = list()
dataFusionInputPs = list()
# =============================== DATA ===================================================
# LocSys client process
# LocStR, LocStS = Pipe(duplex = False)           # LocSys  ->  brain
# from data.localisationsystem.locsys import LocalisationSystemProcess
# LocSysProc = LocalisationSystemProcess([], [LocStS])
# allProcesses.append(LocSysProc)

lsFzzR, lsFzzS = Pipe(duplex=False)
tlFzzR, tlFzzS = Pipe(duplex=False)
imuFzzR, imuFzzS = Pipe(duplex=False)

locsysProc = LocSysSIM([], [lsFzzS])
trafficProc = TrafficSIM([], [tlFzzS])
pathplanProc = PathPlanningProcess([], [], req_path=("13", "111"))
# imuProc = IMUProcess([], [imuFzzS])

allProcesses.append(locsysProc)
allProcesses.append(trafficProc)
# allProcesses.append(imuProc)

# Pipes:
# Camera process -> Lane keeping
lkR, lkS = Pipe(duplex=False)

# Lane keeping -> Movement control
FzzMcR, FzzMcS = Pipe(duplex=False)

# Lane keeping -> Data Fusion
lkFzzR, lkFzzS = Pipe(duplex=False)

# Intersection Detection -> Data Fusion
iDFzzR, iDFzzS = Pipe(duplex=False)

# Movement control -> Serial handler
cfR, cfS = Pipe(duplex=False)

dataFusionInputPs.append(lkFzzR)
dataFusionInputPs.append(iDFzzR)
dataFusionInputPs.append(lsFzzR)
dataFusionInputPs.append(tlFzzR)
# TODO: tech debt
# dataFusionInputPs.append(imuFzzR)

# =============================== RC CONTROL =================================================
if enableRc:
    rcShR, rcShS = Pipe(duplex=False)  # rc      ->  serial handler

    # Serial handler or Simulator Connector
    if enableSIM:
        shProc = SimulatorConnector([cfR], [])
    # else:
    # shProc = SerialHandlerProcess([cfR], [])

    rcProc = RemoteControlReceiverProcess([], [rcShS])
    allProcesses.append(rcProc)

# ===================================== LANE KEEPING  ===================================
if enableLaneKeeping:
    if enableStream:
        lkStrR, lkStrS = Pipe(duplex=False)
        lkProc = LaneKeeping([lkR], [lkFzzS, lkStrS])
    camOutPs.append(lkS)
    movementControlR.append(FzzMcR)
    lkProc = LaneKeeping([lkR], [lkFzzS])
    allProcesses.append(lkProc)

    # Movement control
    cfProc = MovementControl(movementControlR, [cfS])
    allProcesses.append(cfProc)

    # Serial handler or Simulator Connector
    if enableSIM:
        shProc = SimulatorConnector([cfR], [])
    # else:
    # shProc = SerialHandlerProcess([cfR], [])

    allProcesses.append(shProc)

if enableIntersectionDet:
    camiDR, camiDS = Pipe(duplex=False)
    camOutPs.append(camiDS)
    idProc = IntersectionDetProcess([camiDR], [iDFzzS])
    allProcesses.append(idProc)

datafzzProc = DataFusionProcess(dataFusionInputPs, [])
allProcesses.append(datafzzProc)

# ========================= Streamer =====================================================
if enableStream:
    if enableLaneKeeping:
        streamProc = CameraStreamerProcess([lkStrR], [])
    else:
        camStR, camStS = Pipe(duplex=False)  # camera  ->  streamer
        camOutPs.append(camStS)
        streamProc = CameraStreamerProcess([camStR], [])

    allProcesses.append(streamProc)

# ========================== Camera process ==============================================
if enableCameraSpoof:
    camSpoofer = CameraSpooferProcess([], camOutPs, "vid")
    allProcesses.append(camSpoofer)
else:
    if enableSIM:
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
