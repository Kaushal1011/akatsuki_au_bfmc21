import time

class BehaviourCallback:
    
    def __init__(self):
        pass
    
    def out_condition(self) -> bool:
        raise False
    
    def toggle_condition(self) -> bool:
        return False
    
    def __call__(self, car_state):
        raise NotImplementedError
    
    def set(self):
        raise NotImplementedError


class StopBehvaiour(BehaviourCallback):
    
    def __call__(self,car_state):
        # return 0 speed and 0 steering
        return {"speed":0.0, "steering":0.0}

class PriorityBehaviour(BehaviourCallback):
    def __call__(self,car_state):
        # return pririty speed 
        return {"speed":self.state.priority_speed}

class OvertakeBehaviour(BehaviourCallback):
    
    def __init__(self,**kwargs):
        self.overtakepath=None
        
    def reset(self,car_state):
        # set path here for changing lanes
        pass
    
    def __call__(self,car_state):
        pass
        

    
class ActionBehaviour:
    
    def __init__(self,name,release_time=0.0,callback=None):
        self.state = False
        self.state_start = None
        self.action_time = None
        self.release_time = release_time
        self.name = name
        self.callback=callback
        
    def reset(self):
        self.state = False
        self.state_start = None
        self.action_time = None
        # self.release_time = release_time
        # self.name = name
        
    def __call__(self, car_state=None):
        state,toggle=self.state_check()
        if state:
            if self.callback.out_condition():
                self.state=False
            return self.callback(car_state) 
        elif toggle:
            self.callback.toggle_condition()
        else:
            return None
        
    
    def state_check(self):
        if self.state == True:
            if self.action_time is not None:
                if (time.time() - self.state_start) > self.action_time:
                    print("in false state toggle")
                    self.state = False
                    return self.state, True
            return self.state, False
        else:
            return self.state, False

    def set(self, action_time=None,**kwargs):
        if not self.state_start or (self.state_start + self.action_time + self.release_time < time.time()):
            self.action_time = action_time
            self.state_start = time.time()
            self.state=True
            self.state.callback.set(**kwargs)
            print("State set")
            return self.state
        else:
            return self.state