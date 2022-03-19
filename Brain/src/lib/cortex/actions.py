import time


class Action:
    def __init__(self, name, release_time) -> None:
        self.state = False
        self.state_start = None
        self.action_time = None
        self.release_time = release_time
        self.name = name

    def state_check(self):
        if self.state == True:
            if (time.time() - self.state_start) > self.action_time:
                print("in false state toggle")
                self.state = False, False
                return False, True
            print("in true toggle")
            return True, False
        else:
            return False, False

    def set(self, action_time):

        if not self.state_start or (self.state_start + self.action_time + self.release_time < time.time()):
            self.action_time = action_time
            self.state_start = time.time()
            self.state=True
            print("in here")
            return True
        else:
            return False

