#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, Point, WrenchStamped, Wrench, Vector3
from std_msgs.msg import Header,Empty
import tf
import numpy as np

class ZmpPub:
  def __init__(self, **kwargs):
    self.nn = rospy.init_node('zmpPublisher')
    self.zmpPub = rospy.Publisher("/robotv1/ZMP/point", PointStamped, queue_size=10)
    self.frame_id = 'hello'
    self.rate = rospy.Rate(10)
    self.cc = 0
    self.force_array = np.zeros(3)
    self.torque_array = np.zeros(3)
 
  def Vector3_to_array(self,vector):
    Array = np.zeros(3)
    Array[0] = vector.x
    Array[1] = vector.y
    Array[2] = vector.z
    return Array
  
  def array_to_Vector3(self,Array):
    vector3 = Vector3(x=Array[0],y=Array[1],z=Array[2])
    return vector3

  def callback(self, data): # data is WrenchStamped Type
    self.force_array = self.force_array + self.Vector3_to_array(data.wrench.force)
    self.torque_array = self.torque_array + self.Vector3_to_array(data.wrench.torque)
    self.frame_id = data.header.frame_id  
    self.cc = self.cc + 1

  def publishZmp(self):
    while not rospy.is_shutdown():
      d = 0.005
      self.zmpsub = rospy.Subscriber("/robotv1/sensor/ft_sensor/LLEG_ft",WrenchStamped, self.callback)
  # if no msg has been subscribed  
      print("force after subscribe")
      print(self.force_array)
      print("after subscribe, this is self.cc")
      print(self.cc)
      if self.cc != 0: 
        self.force_array = self.force_array/self.cc
        self.torque_array = self.torque_array/self.cc
      print("average of force")
      print(self.force_array)
  # if their is no contact force on it, meaming fz = 0, then send empty message
      if abs(self.force_array[2])<1e-5:
        zmpStamped = PointStamped()
      else:
        zmpx = (-self.torque_array[1]-self.force_array[0]*d)/self.force_array[2]
        zmpy = (self.torque_array[0]-self.force_array[1]*d)/self.force_array[2]
        PStamped = Point(x=zmpx, y=zmpy, z=-d)
       
        Time = rospy.get_rostime()
        H = Header(stamp=Time, frame_id=self.frame_id)
        zmpStamped = PointStamped(header=H, point=PStamped)  
      
      PStamped_msg = zmpStamped

      print("helloworld")
      self.zmpPub.publish(PStamped_msg)

      self.force_array = np.zeros(3)
      self.torque_array = np.zeros(3)
      self.cc = 0

      self.rate.sleep()

if __name__== "__main__":
  try:
    zmpNodePub = ZmpPub()
    zmpNodePub.publishZmp()
  except rospy.ROSInterruptException:
    pass


