#!/usr/bin/python3
import rospy, rospkg, sys, numpy as np
sys.dont_write_bytecode = True

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

class Joystick:
    def __init__(self, model, topic):
        
        mapping = np.load(rospkg.RosPack().get_path('drlarac_vsss')+'/scripts/modules/joysticks.npz', allow_pickle=True)
        self.buttons_keys, self.axes_keys = mapping[model].item().values()
        
        self.buttons = dict.fromkeys(self.buttons_keys, 0)
        self.axes = dict.fromkeys(self.axes_keys, 0.0)
        
        rospy.Subscriber(topic, Joy, self.joystick_callback, queue_size=1)
        
        self.observers = []
        
    def joystick_callback(self, msg:Joy):
        buttons_values = msg.buttons
        axes_values = msg.axes
        
        self.buttons = dict(zip(self.buttons_keys, buttons_values))
        self.axes = dict(zip(self.axes_keys, axes_values))
        
        self.notify_observers()
        
    def register_observer(self, observer):
        self.observers.append(observer)
        
    def notify_observers(self):
        for observer in self.observers:
            observer.update()

class JoystickVSSS:
    def __init__(self, model, topic, mode, r, b, color_team="yellow"):
        self.joystick = Joystick(model=model, topic=topic)
        self.joystick.register_observer(self)
        
        self.mode = mode
        self.r, self.b = r, b
        self.vel = 30
        self.next_robot_button = 0
        self.prev_robot_button = 0
        
        rospy.loginfo(f'Mode: {mode}')
        
        # Teams
        self.team = []
        for i in range(3):
            self.team.append([rospy.Publisher(f'/{color_team}/{i}/left_controller/command', Float64, queue_size=1), 
                         rospy.Publisher(f'/{color_team}/{i}/right_controller/command', Float64, queue_size=1)])

        self.current_id = 0
        
    def update(self):
        if self.mode == 'direct': self.direct_mode()
        elif self.mode == 'diff': self.diff_mode()
        else: 
            rospy.logerr('Invalid mode')
            return
        
        if self.joystick.buttons['BACK']:
            self.reset_world()
        
        if self.joystick.buttons['X']:
            rospy.loginfo('Team blue')
            self.current_team = 'blue'
            self.current_id = 0
        if self.joystick.buttons['Y']:
            rospy.loginfo('Team yellow')
            self.current_team = 'yellow'
            self.current_id = 0
            
        if self.joystick.buttons['RB']:
            if self.next_robot_button: return
            if self.current_id < 2: self.current_id += 1
            else:  self.current_id = 0
            rospy.loginfo(f'Robot ID: {self.current_id}')
        
        if self.joystick.buttons['LB']:
            if self.prev_robot_button: return
            if self.current_id > 0: self.current_id -= 1
            elif self.current_id == 0: self.current_id = 2
            rospy.loginfo(f'Robot ID: {self.current_id}')
        
        self.next_robot_button, self.prev_robot_button = self.joystick.buttons['RB'], self.joystick.buttons['LB']
        
    def direct_mode(self):
        self.team[self.current_id][0].publish(self.joystick.axes['LV']*self.vel)
        self.team[self.current_id][0].publish(self.joystick.axes['RV']*self.vel)
        
    def diff_mode(self):
        V = self.joystick.axes['LV']*0.75
        W = self.joystick.axes['RH']*20.0
        
        # V = (self.joystick.axes['RT']-1)/-2*0.75+(self.joystick.axes['LT']-1)/-2*-0.75
        # W = self.joystick.axes['LH']*10.0
        
        wl = (2*V-self.b*W)/(60*self.r)*self.vel
        wr = (2*V+self.b*W)/(60*self.r)*self.vel
        
        wl, wr = np.clip([wl, wr], -self.vel, self.vel)
        
        self.team[self.current_id][0].publish(wl)
        self.team[self.current_id][1].publish(wr)
        
        # self.robots[self.current_team][self.current_id][0][0].publish(wl)
        # self.robots[self.current_team][self.current_id][1][0].publish(wr)
    
def main():
    rospy.init_node('joycom2')
    node = JoystickVSSS(model='Machenike', topic='/joy2', mode='diff', r=0.025, b=0.075)
    rospy.spin()

if __name__ == '__main__':
    main()