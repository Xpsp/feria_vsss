#!/usr/bin/python3
import rospy, sys, struct, serial
sys.dont_write_bytecode = True

from std_msgs.msg import Float64
import numpy as np

class Comunication:
    def __init__(self, num_connections):
        
        self.serial = serial.Serial('/dev/ttyUSB0', 115200)
        # self.serial = serial.Serial('/dev/ttyACM0', 115200)
        self.team = [[0.0, 0.0] for _ in range(3)]
        
        self.num_connections = num_connections
        
        for i in range(num_connections):
            for side in ['left', 'right']:
                for color in ['blue', 'yellow']:
                    topic = f'/{color}/{i}/{side}_controller/command'
                    callback = getattr(self, f'team_{i}_{side}_callback')
                    rospy.Subscriber(topic, Float64, callback, queue_size=1)
                    
        # rospy.Timer(rospy.Duration(0.1), self.comunication_callback)
    
    def send_message(self, message):
        packed_data = struct.pack('>BHHHHHHBB', *message)
        self.serial.write(packed_data)
        
    def team_0_left_callback(self, msg:Float64): 
        self.team[0][0]= msg.data
    def team_0_right_callback(self, msg:Float64): 
        self.team[0][1] = msg.data
        if self.num_connections >= 1: 
            self.comunication_callback(0)
    def team_1_left_callback(self, msg:Float64): 
        self.team[1][0] = msg.data
    def team_1_right_callback(self, msg:Float64): 
        self.team[1][1] = msg.data
        if self.num_connections >= 2: 
            self.comunication_callback(1)
    def team_2_left_callback(self, msg:Float64): 
        self.team[2][0] = msg.data
    def team_2_right_callback(self, msg:Float64): 
        self.team[2][1] = msg.data
        if self.num_connections >= 3:
            self.comunication_callback(2)
    
    def comunication_callback(self, idx):
        resolution = 65535
        vels = [(np.int32(np.abs(self.team[i][0]*resolution//30)), np.int32(np.abs(self.team[i][1]*resolution//30))) for i in range(self.num_connections)]
        dirs = sum([(self.team[i][0]<0)<<(2*i)|(self.team[i][1]<0)<<(2*i+1) for i in range(self.num_connections)])
        msg = [253] + [w if w not in (253, 254) else 255 for vel in vels for w in vel] + [dirs] + [254]
        self.send_message(msg)
        print(f'{idx}: {msg}')
        
        
def main():
    rospy.init_node('communication')
    node = Comunication(3)
    rospy.spin()
    

if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass