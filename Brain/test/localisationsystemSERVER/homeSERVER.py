from multiprocessing import Event

from homedata import LocalisationServer

# ===================================== MAIN =============================================
if __name__ == "__main__":
    a = LocalisationServer()
    a.daemon = True
    a.start()
    blocker = Event()
    try:
        blocker.wait()
    except KeyboardInterrupt:
        print("\nCatching a KeyboardInterruption exception! Shutdown all processes.")
