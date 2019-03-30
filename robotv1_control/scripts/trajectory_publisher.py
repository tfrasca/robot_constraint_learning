#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPub:
  def __init__(self,**kwargs):
    self.nn = rospy.init_node('trajectoryPublisher')
    self.RLEG_trajPub = rospy.Publisher("/robotv1/RLEG_trj_controller/command", JointTrajectory, queue_size=10)
    self.rate = rospy.Rate(10)
    self.RLEGJointNames = ['RLEG_H0','RLEG_H1', 'RLEG_H2','RLEG_K0','RLEG_A0','RLEG_A1']

    self.LLEG_trajPub = rospy.Publisher("/robotv1/LLEG_trj_controller/command", JointTrajectory, queue_size=10)
    self.LLEGJointNames = ['LLEG_H0','LLEG_H1', 'LLEG_H2','LLEG_K0','LLEG_A0','LLEG_A1']

  def publishTrajectory(self):
    trajectoryPositions = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
    Velocity = [0, 0, 0, 0, 0, 0]
    Effort = [10, 10, 10, 10, 10, 10]
    Acceleration = [0, 0, 0, 0, 0, 0]
    TimeFromStart = rospy.Time(3,0)
    
    while not rospy.is_shutdown():
      if trajectoryPositions == []:
        trajPoses = []  
      else:
        trajPoses = [JointTrajectoryPoint(positions = trajectoryPositions, velocities = Velocity, accelerations = Acceleration, time_from_start = TimeFromStart, effort=Effort)]

      RLEG_traj_msg = JointTrajectory(joint_names = self.RLEGJointNames, points = trajPoses)
      self.RLEG_trajPub.publish(RLEG_traj_msg)
      
      LLEG_traj_msg = JointTrajectory(joint_names = self.LLEGJointNames, points = trajPoses)
      self.LLEG_trajPub.publish(LLEG_traj_msg)

      self.rate.sleep()

if __name__ == "__main__":
  try:
    trajNodePub = TrajectoryPub()
    trajNodePub.publishTrajectory()

  except rospy.ROSInterruptException:
    pass
