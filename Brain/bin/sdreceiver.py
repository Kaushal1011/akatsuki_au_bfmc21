
from multiprocessing import Event

from src.utils.camerastreamer.CameraReceiverProcess import CameraReceiverProcess

# ===================================== MAIN =============================================
if __name__ == "__main__":
    a = CameraReceiverProcess([], [], 4422)
    a.start()
    blocker = Event()
    try:
        blocker.wait()
    except KeyboardInterrupt:
        print("\nCatching a KeyboardInterruption exception! Shutdown all processes.")
        a.terminate()
    a.join()