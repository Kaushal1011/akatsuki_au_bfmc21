import time
from src.lib.cortex.control_sys import Pure_Pursuit
import math
import joblib

from src.lib.cortex.carstate import CarState
import pathlib
data_path = pathlib.Path(pathlib.Path(__file__).parent.parent.parent.resolve(),"data", "mid_course.z" )
data=joblib.load(data_path)
ptype=data[1]
etype=data[2]
class BehaviourCallback:
    
    def __init__(self):
        pass
    
    def out_condition(self) -> bool:
        return False
    
    def toggle_condition(self) -> bool:
        return False
    
    def __call__(self, car_state):
        raise NotImplementedError
    
    def reset(self):
        pass
    
    def set(self):
        raise NotImplementedError


class StopBehvaiour(BehaviourCallback):
    
    def __call__(self,car_state):
        # return 0 speed and 0 steering
        return {"speed":0.0, "steer":0.0}
    
    def set(self):
        pass

class PriorityBehaviour(BehaviourCallback):
    def __call__(self,car_state):
        # return pririty speed 
        return {"speed":self.state.priority_speed}

    def set(self):
        pass

class OvertakeBehaviour(BehaviourCallback):

    def reset(self):
        # set path here for changing lanes
        pass
    
    def __call__(self,car_state):
        pass
   
    def set(self):
        pass
        
class LaneKeepBehaviour(BehaviourCallback):
    def __call__(self, car_state):
        if abs(car_state.cs_steer - car_state.lanekeeping_angle)>20:
            return None
        elif car_state.current_ptype == "lk":
            # return  {"steer":car_state.lanekeeping_angle}
            return None
    
    def set(self):
        pass

class ControlSystemBehaviour(BehaviourCallback):

    def __init__(self,coord_list):
        super().__init__()
        self.cs=Pure_Pursuit(coord_list)
    
    def __call__(self, car_state:CarState):
        ind,lf=self.cs.search_target_index(car_state)
        print("Loc Index: ",ind)
        print("Target: ",self.cs.cx[ind]," ", self.cs.cy[ind])
        car_state.current_ptype=ptype[ind]
        car_state.can_overtake=etype[ind]
        di=self.cs.purest_pursuit_steer_control(car_state, ind, lf)
        di = di * 180 / math.pi
        if di > 23:
            di = 23
        elif di < -23:
            di = -23
        car_state.cs_steer=di
        return {"steer":di, "speed":car_state.max_v}

    def set(self):
        pass
    
    def reset(self,**kwargs):
        self.cs.reset(kwargs["coord_list"])
        return True


class ObjectStopBehaviour(BehaviourCallback):
    def __call__(self, car_state):
        if car_state.front_distance<0.3:
            return {'speed':0.0}
    
    def set(self):
        pass

    
class ActionBehaviour:
    
    def __init__(self,name,release_time=0.0,callback=None):
        self.state = False
        self.state_start = None
        self.action_time = None
        self.release_time = release_time
        self.name = name
        self.callback=callback
        
    def reset(self,**kwargs):
        self.state = False
        self.state_start = None
        self.action_time = None
        self.callback.reset(**kwargs)
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
            return {"toggle":True}
        else:
            return {}
        
    
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
            self.callback.set(**kwargs)
            print("State set")
            return self.state
        else:
            return self.state
    
    def check_cooldown(self):
        
        if not self.state_start or (self.state_start + self.action_time + self.release_time < time.time()):
            print("Check Cooldown Called: ",True)
            return True
        else:
            return False



class ActionManager:

    def __init__(self):
        self.lk=None
        self.cs=None
        self.objstop=None
        self.l1_ab=None
        self.l2_ab=None
        self.l3_ab=None
        self.l4_ab=None
    
    def __call__(self,carstate):

        obj=[self.cs,self.lk,self.l1_ab,self.l2_ab,self.l3_ab,self.l4_ab,self.objstop]
        speed=0
        steer=0
        count=0

        #reset cs after interrupt using either self

        for i in obj:   
            if i:
                outputs=i(carstate)
                if outputs and "speed" in outputs.keys():
                    speed=outputs["speed"]
                if outputs and "steer" in outputs.keys():
                    steer=outputs["steer"]
                if outputs and "interrupt" in outputs.keys():
                    self.cs.reset(coord_list=outputs["interrupt"])
                if outputs and "toggle" in outputs.keys():
                    if count==2:
                        self.l1_ab=None
                    elif count==3:
                        self.l2_ab=None
                    elif count==5:
                        self.l4_ab=None
            count+=1

        return speed,steer

    def set_action(self,action,action_time=None,**kwargs):
        if action.name=="cs":
            self.cs=action
            self.cs.set(action_time=action_time,**kwargs)
            return True
        elif action.name=="lk":
            self.lk=action
            self.lk.set(action_time=action_time,**kwargs)
            return True
        elif action.name=="hw" or action.name=="roundabout":
            self.l1_ab=action
            self.l1_ab.set(action_time=action_time,**kwargs)
            return True
        elif action.name=="parking" or action.name=="overtaking" or action.name=="tailing":
            self.l2_ab=action
            self.l2_ab.set(action_time=action_time,**kwargs)
            return True
        elif (action.name=="stop" or action.name=="priority" or action.name=="crosswalk") and (self.l3_ab is None or self.l3_ab.check_cooldown()): # or action.name!=self.l3_ab.name):
            self.l3_ab=action
            self.l3_ab.set(action_time=action_time,**kwargs)
            return True
        elif action.name=="ramp" or action.name=="interrupt":
            self.l4_ab=action
            self.l4_ab.set(action_time=action_time,**kwargs)
            return True
        elif action.name=="objstop":
            self.objstop=action
            self.objstop.set(action_time=action_time,**kwargs)
            return True
        else:
            return False
        
    

