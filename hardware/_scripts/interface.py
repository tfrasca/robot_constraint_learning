#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
# import Adafruit_PCA9685

class Interface:
    def __init__(self, **kwargs):
        self.pos = np.zeros(14)
        self.name = []
        self.frequency = 60

        self.start_pulse = int(4096*self.frequency*0.5//1000)
       # self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=2)
        self.Pin = {
            "LLEG_A0": 4,
            "LLEG_A1": 5,
            "LLEG_H0": 0,
            "LLEG_H1": 1,
            "LLEG_H2": 2,
            "LLEG_K0": 3,
            "LLEG_ft_sensor": -1,
            
            "RLEG_A0": 10,
            "RLEG_A1": 11,  
            "RLEG_H0": 6,
            "RLEG_H1": 7,
            "RLEG_H2": 8,
            "RLEG_K0": 9,
            "RLEG_ft_sensor": -1 }
        self.calib = {
            "LLEG_A0": [1, np.pi/2],
            "LLEG_A1": [1, np.pi/2],
            "LLEG_H0": [1, np.pi/2],
            "LLEG_H1": [-1, np.pi/2],
            "LLEG_H2": [-1, np.pi/2],
            "LLEG_K0": [-1, np.pi],
            "LLEG_ft_sensor": [0, 0],
            
            "RLEG_A0": [-1, np.pi/2],
            "RLEG_A1": [1, np.pi/2],  
            "RLEG_H0": [-1, np.pi/2],
            "RLEG_H1": [-1, np.pi/2],
            "RLEG_H2": [-1, np.pi/2],
            "RLEG_K0": [1, 0],  
            "RLEG_ft_sensor": [0, 0] }

    # get the joint position and name from topic
    def callback(self, data): 
        if self.name == []:
            self.name = data.name
        self.pos = np.asarray(data.position)

    # calibrate the difference of the joint direction and limitation between real robot and simulation(URDF) 
    def calibration(self,pos):
        for i in range(len(self.name)):
            pos[i] = pos[i]*self.calib[self.name[i]][0] + self.calib[self.name[i]][1] 
        #print("calibration done!!!")
        return pos

    # convert the angle data to the pulse bit, resolution 4096
    def rad_to_pulse(self,pos):
        pos = (2*pos/np.pi + 1)*4096*self.frequency//1000 
        #print("transmation done!!!")
        return pos
    
    def get_pos(self):
        pos = self.pos
        pos = self.rad_to_pulse(self.calibration(pos)) 
        return np.floor(pos) 
        

    # send command to the controller 
    def execute(self,pos):
        for i in range(len(self.name)):
            continue
            # print(self.name[i],"pin = ", self.Pin[self.name[i]], "  pos = ", int(pos[i]))    
           # self.pwm.set_pwm(self.Pin[self.name[i]],self.start_pulse, int(pos[i]))
    


if __name__ == '__main__':
    try:
        interface = Interface()
        nh = rospy.init_node('robot_execute', anonymous=True)
        rate = rospy.Rate(20)
        rospy.Subscriber('/robotv1/joint_states', JointState,interface.callback)
        pos0 = interface.get_pos()
        count = 0
        while not rospy.is_shutdown():
            if count < 10:
                pos0 = interface.get_pos()  
            else:
                pos = interface.get_pos()
                pos_max = max(abs(pos-pos0))
                if (pos_max > 1) & (pos_max < 300):
                    rospy.loginfo("Moving! "+ str(count))
                    interface.execute(pos)
                    pos0 = pos
                else: 
                    print("posx_max = ", pos_max, "Drop execution!!!")

            count += 1
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
        
        
