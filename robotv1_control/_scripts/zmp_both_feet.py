#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import PointStamped, Point, WrenchStamped, Wrench, Vector3,TransformStamped, Quaternion
from std_msgs.msg import Header,Empty
import tf
import tf2_ros
import numpy as np

class ZmpPub:
  def __init__(self, **kwargs):
    self.nn = rospy.init_node('zmp_bot_feet_publisher')
    self.ZMP_Pub = rospy.Publisher("/robotv1/ZMP/ZMP_point", PointStamped, queue_size=1)
    self.ZMP_ft_Pub = rospy.Publisher("/robotv1/ZMP/ZMP_ft", WrenchStamped, queue_size=1)
       
    self.br = tf2_ros.TransformBroadcaster()
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.rate = rospy.Rate(10)
    self.d = 0.005  # distance between ft_sensor_joint and the groung along z-axis 
    self.threshold = 0.1
    self.ZMP_frame = 'ZMP'
    self.ZMP_transform = self.init_transform()
    self.ZMP_ft = Wrench()

    self.LLEG_zmp_frame = 'LLEG_zmp'
    self.RLEG_zmp_frame = 'RLEG_zmp'
    
    self.LLEG_zmp_ft = Wrench()
    self.RLEG_zmp_ft = Wrench()
    

  def Vector3_to_array(self,vector):
    Array = np.zeros(3)
    Array[0] = vector.x
    Array[1] = vector.y
    Array[2] = vector.z
    return Array
  
  def array_to_Vector3(self,Array):
    vector3 = Vector3(x=Array[0],y=Array[1],z=Array[2])
    return vector3


  def LLEG_zmp_ft_callback(self, data):
    self.LLEG_zmp_ft = data.wrench
  
  def RLEG_zmp_ft_callback(self, data):
    self.RLEG_zmp_ft = data.wrench
  
  def init_transform(self):
    Time = rospy.Time.now()
    H = Header(stamp=Time, frame_id='dummy')
    trans = TransformStamped(header=H)
    trans.child_frame_id=self.ZMP_frame
    trans.transform.rotation.w=1
    return trans



  def ZMP_transform_msg(self):
    Lforce_array = self.Vector3_to_array(self.LLEG_zmp_ft.force)   
    Rforce_array = self.Vector3_to_array(self.RLEG_zmp_ft.force)
    Ltorque_array = self.Vector3_to_array(self.LLEG_zmp_ft.torque)   
    Rtorque_array = self.Vector3_to_array(self.RLEG_zmp_ft.torque)
    self.ZMP_ft.force = self.array_to_Vector3(Lforce_array + Rforce_array)
    self.ZMP_ft.torque = self.array_to_Vector3(Ltorque_array + Rtorque_array)
    
    try:
      Now = rospy.Time.now()
      L_transform = TransformStamped()
      L_transform.transform.rotation.w = 1
      R_transform = TransformStamped()
      R_transform.transform.rotation.w = 1
      
      L_transform = self.tfBuffer.lookup_transform('dummy', self.LLEG_zmp_frame, Now, rospy.Duration(1.0))
      R_transform = self.tfBuffer.lookup_transform('dummy', self.RLEG_zmp_frame, Now, rospy.Duration(1.0))
 

      if (abs(Lforce_array[2]>self.threshold)) and (abs(Rforce_array[2]<self.threshold)):
        self.ZMP_transform = L_transform
      elif (abs(Lforce_array[2])<self.threshold) and (abs(Rforce_array[2])>self.threshold):
        self.ZMP_transform = R_transform
      
      elif (abs(Lforce_array[2])<self.threshold) and (abs(Rforce_array[2])<self.threshold):
        self.ZMP_transform = L_transform
        self.ZMP_transform.transform.translation = Vector3(x=0,y=0,z=0)
        self.ZMP_transform.transform.rotation.w = 1
      else:
        Ltrans = np.zeros(3)
        Rtrans = np.zeros(3)
        zmp_trans = np.zeros(3)
 
        Ltrans = self.Vector3_to_array(L_transform.transform.translation)
        Rtrans = self.Vector3_to_array(R_transform.transform.translation)
 
        zmp_trans[0] = (Ltrans[0]*Lforce_array[2]+ Rtrans[0]*Rforce_array[2])/(Lforce_array[2] + Rforce_array[2])
        zmp_trans[1] = (Ltrans[1]*Lforce_array[2]+ Rtrans[1]*Rforce_array[2])/(Lforce_array[2] + Rforce_array[2])
        zmp_trans[2] = (Ltrans[2]+Rtrans[2])/2.0
      
        self.ZMP_transform = L_transform
        self.ZMP_transform.transform.translation = self.array_to_Vector3(zmp_trans)
        self.ZMP_transform.child_frame_id = self.ZMP_frame

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      self.rate.sleep()
      pass 
    
    msg = self.ZMP_transform
    return msg


  def ZMP_msg(self):
    Time = rospy.Time.now()
    H = Header(stamp=Time, frame_id=self.ZMP_frame)
    msg = PointStamped(header=H, point=Point(x=0,y=0,z=0))
    return msg

  def ZMP_ft_msg(self):
    Time = rospy.Time.now()
    H = Header(stamp=Time, frame_id=self.ZMP_frame)
    msg = WrenchStamped(header=H,wrench=self.ZMP_ft)
    return msg

  def publishZmp(self):
    while not rospy.is_shutdown():
      rospy.Subscriber("/robotv1/ZMP/LLEG_zmp_ft",WrenchStamped, self.LLEG_zmp_ft_callback)
      rospy.Subscriber("/robotv1/ZMP/RLEG_zmp_ft",WrenchStamped, self.RLEG_zmp_ft_callback)
      
      self.br.sendTransform(self.ZMP_transform_msg())
         
      
      self.ZMP_Pub.publish(self.ZMP_msg())
      self.ZMP_ft_Pub.publish(self.ZMP_ft_msg())  

      self.rate.sleep()

if __name__== "__main__":
  try:
    zmpNodePub = ZmpPub()
  
    zmpNodePub.publishZmp()
  except rospy.ROSInterruptException:
    pass

  
