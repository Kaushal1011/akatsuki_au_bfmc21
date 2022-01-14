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
from multiprocessing import Event, Pipe, Process

from src.hardware.camera.cameraprocess import CameraProcess
from src.hardware.camera.CameraSpooferProcess import CameraSpooferProcess
from src.hardware.serialhandler.SerialHandlerProcess import SerialHandlerProcess
from src.utils.camerastreamer.CameraStreamerProcess import CameraStreamerProcess
from src.utils.controlsys.lanekeeping import LaneKeepingProcess as LaneKeeping

# ========================================================================
# SCRIPT USED FOR WIRING ALL COMPONENTS
# ========================================================================
from src.utils.controlsys.momentcontrol import MovementControl
from src.utils.remotecontrol.RemoteControlReceiverProcess import (
    RemoteControlReceiverProcess,
)

sys.path.append(".")


# hardware imports

# utility imports

# =============================== CONFIG =================================================
enableStream = True
enableCameraSpoof = False
enableRc = False
enableLaneKeeping = True

# =============================== INITIALIZING PROCESSES =================================
# Pipe collections
allProcesses = list()
movementControlR = list()
camOutPs = list()

# =============================== DATA ===================================================
# LocSys client process
# LocStR, LocStS = Pipe(duplex = False)           # LocSys  ->  brain
# from data.localisationsystem.locsys import LocalisationSystemProcess
# LocSysProc = LocalisationSystemProcess([], [LocStS])
# allProcesses.append(LocSysProc)

# Pipes:
# Camera process -> Lane keeping
lkR, lkS = Pipe(duplex=False)

# Lane keeping -> Movement control
lcR, lcS = Pipe(duplex=False)

# Movement control -> Serial handler
cfR, cfS = Pipe(duplex=False)

# =============================== RC CONTROL =================================================
if enableRc:
    rcShR, rcShS = Pipe(duplex=False)  # rc      ->  serial handler

    # serial handler process
    shProc = SerialHandlerProcess([rcShR], [])
    allProcesses.append(shProc)

    rcProc = RemoteControlReceiverProcess([], [rcShS])
    allProcesses.append(rcProc)

# ===================================== LANE KEEPING  ===================================
if enableLaneKeeping:
    if enableStream:
        lkStrR, lkStrS = Pipe(duplex=False)

    camOutPs.append(lkS)
    movementControlR.append(lcR)
    lkProc = LaneKeeping([lkR], [lcS, lkStrS])
    allProcesses.append(lkProc)

    # Movement control
    cfProc = MovementControl(movementControlR, [cfS])
    allProcesses.append(cfProc)

    # Serial handler
    shProc = SerialHandlerProcess([cfR], [])
    allProcesses.append(shProc)

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
