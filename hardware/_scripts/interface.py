#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def callback(data):
    print("hello", data)
   #  rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.
    rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
