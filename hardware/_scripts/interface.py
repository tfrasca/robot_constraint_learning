#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
# import Adafruit_PCA9685

class Interface:
    def __init__(self, **kwargs):
        self.nh = rospy.init_node('robot_execute', anonymous=True)
        self.jnt_states = JointState()
        self.rate = rospy.Rate(5)
        self.pos = []
        self.name = []
        self.frequency = 60
        self.start_pulse = 4096*self.frequency*0.5//1000
       # self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=2)
        self.Pin = {
            "LLEG_A0": 4,
            "LLEG_A1": 5,
            "LLEG_H0": 0,
            "LLEG_H1": 1,
            "LLEG_H2": 2,
            "LLEG_K0": 3,
            
            "RLEG_A0": 10,
            "RLEG_A1": 11,  
            "RLEG_H0": 6,
            "RLEG_H1": 7,
            "RLEG_H2": 8,
            "RLEG_K0": 9  }
        self.calib = {
            "LLEG_A0": [1, np.pi/2],
            "LLEG_A1": [1, np.pi/2],
            "LLEG_H0": [1, np.pi/2],
            "LLEG_H1": [-1, np.pi/2],
            "LLEG_H2": [-1, n,pi/2],
            "LLEG_K0": [-1, np.pi],
            
            "RLEG_A0": [-1, np.pi/2],
            "RLEG_A1": [1, np.pi/2],  
            "RLEG_H0": [-1, np.pi/2],
            "RLEG_H1": [-1, np.pi/2],
            "RLEG_H2": [-1, np.pi/2],
            "RLEG_K0": [1, 0]  }

    # get the joint position and name from topic
    def callback(self, data): 
        if self.name == []:
            self.name = data.name
        self.pos = data.position
    
    # calibrate the difference of the joint direction and limitation between real robot and simulation(URDF) 
    def calibration(self):
        self.pos = np.asarray(self.pos)
        for i in range(len(self.name)):
            self.pos[i] = self.pos[i]*self.calib[self.name[i]][0] + self.calib[self.name[i]][1] 
        print("after calibration",self.pos)

    # convert the angle data to the pulse bit, resolution 4096
    def rad_to_pulse(self):
        self.pos = (2*self.pos/np.pi + 1)*4096*self.frequency//1000 
    
    # send command to the controller 
    def execute(self):
        for i in range(len(self.name)):
            print(self.name[i],"pin = ", self.Pin[self.name[i]], "  pos = ", self.pos[i])    
           # pwm.set_pwm(self.Pin[self.name[i]],self.start_pulse, self.pos[i])
    
    # main function 
    def listener(self):
        while not rospy.is_shutdown():
            rospy.Subscriber('/move_group/fake_controller_joint_states', JointState,self.callback)
            if len(self.name) !=0 :
                self.calibration()
                self.rad_to_pulse()
                self.execute()
            else:
                continue

            self.rate.sleep()
  
if __name__ == '__main__':
    try:
        interface = Interface()
        interface.listener()

    except rospy.ROSInterruptException:
        pass
        
        
