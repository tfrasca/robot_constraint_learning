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
    self.nn = rospy.init_node('zmpPublisher')
    self.LLEG_zmpPub = rospy.Publisher("/robotv1/ZMP/LLEG_point", PointStamped, queue_size=1)
    self.RLEG_zmpPub = rospy.Publisher("/robotv1/ZMP/RLEG_point", PointStamped, queue_size=1)
    self.LLEG_zmp_ft_Pub = rospy.Publisher("/robotv1/ZMP/LLEG_zmp_ft", WrenchStamped, queue_size=1)
    self.RLEG_zmp_ft_Pub = rospy.Publisher("/robotv1/ZMP/RLEG_zmp_ft", WrenchStamped, queue_size=1)
    self.br = tf2_ros.TransformBroadcaster()

    self.rate = rospy.Rate(10)
    self.d = 0.005  # distance between ft_sensor_joint and the groung along z-axis 
    self.threshold = 0.1
    self.LLEG_ft_sensor_frame = 'LLEG_l4'
    self.RLEG_ft_sensor_frame = 'RLEG_l4'
    self.LLEG_zmp_frame = 'LLEG_zmp'
    self.RLEG_zmp_frame = 'RLEG_zmp'
    self.ZMP_frame = 'ZMP'
    self.ZMP_transform = TransformStamped()
    
    self.reset_arg()
    
    rospy.loginfo("Initialized")
    
    self.force_sensing_threshold = 1000.0
    self.torque_sensing_threshold = 1000.0
    self.count_threshold =  300

  def reset_arg(self):
    self.LLEG_force_array = np.zeros(3)
    self.LLEG_torque_array = np.zeros(3)
    self.LLEG_count = 0
    
    self.RLEG_force_array = np.zeros(3)
    self.RLEG_torque_array = np.zeros(3)
    self.RLEG_count = 0
    
  def Vector3_to_array(self,vector):
    Array = np.zeros(3)
    Array[0] = vector.x
    Array[1] = vector.y
    Array[2] = vector.z
    return Array
  
  def array_to_Vector3(self,Array):
    vector3 = Vector3(x=Array[0],y=Array[1],z=Array[2])
    return vector3

  def RLEG_callback(self, data): # data is WrenchStamped Type
    f =  self.Vector3_to_array(data.wrench.force)  
    t =  self.Vector3_to_array(data.wrench.torque)
    self.RLEG_ft_sensor_frame = data.header.frame_id  
    if self.RLEG_count < self.count_threshold: 
      if (max(abs(f) < self.force_sensing_threshold)) and (max(abs(t)) < self.torque_sensing_threshold):
        self.RLEG_force_array += f
        self.RLEG_torque_array += t
        self.RLEG_count += 1
      else:
        rospy.logwarn("RLEG_Callback, drop the data %s" %data.wrench)

  def LLEG_callback(self, data): # data is WrenchStamped Type
    f =  self.Vector3_to_array(data.wrench.force)  
    t =  self.Vector3_to_array(data.wrench.torque)
    self.LLEG_ft_sensor_frame = data.header.frame_id  
    if self.LLEG_count < self.count_threshold: 
      if (max(abs(f)) < self.force_sensing_threshold) and (max(abs(t)) < self.torque_sensing_threshold):
        self.LLEG_force_array += f
        self.LLEG_torque_array += t
        self.LLEG_count += 1
      else:
        rospy.logwarn("LLEG_Callback, drop the data %s" %data.wrench)
    
  def ft_mean(self):
    if self.LLEG_count != 0: 
      self.LLEG_force_array = self.LLEG_force_array/self.LLEG_count
      self.LLEG_torque_array = self.LLEG_torque_array/self.LLEG_count
    if self.RLEG_count != 0: 
      self.RLEG_force_array = self.RLEG_force_array/self.RLEG_count
      self.RLEG_torque_array = self.RLEG_torque_array/self.RLEG_count
  
  def Cal_zmp_array(self, force_array, torque_array): 
    zmp = np.zeros(3)   
    # if their is no contact force on it, meaming fz = 0, then send empty message
    if abs(force_array[2])<self.threshold:
      zmp = np.array([0,0,-5*self.d])
    else:
      zmp[0] = (-torque_array[1]-force_array[0]*self.d)/force_array[2]
      zmp[1] = (torque_array[0]-force_array[1]*self.d)/force_array[2]
      zmp[2]= -self.d
    return zmp 

  def zmp_msg(self, zmp, ref_frame):
    Time = rospy.get_rostime()
    H = Header(stamp=Time, frame_id=ref_frame)
    
    p = self.array_to_Vector3(zmp)
    msg = PointStamped(header=H, point=p)
    return msg
  
  def zmp_frame_msg(self, zmp_array, target_frame, source_frame):
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = source_frame
    msg.child_frame_id = target_frame
    msg.transform.translation = self.array_to_Vector3(zmp_array)
    msg.transform.rotation.x = 0
    msg.transform.rotation.y = 0
    msg.transform.rotation.z = 0
    msg.transform.rotation.w = 1    
    return msg

    
  def zmp_ft_msg(self, force_array, torque_array, ref_frame):
    torque_array[0:2]= 0.0 
    Time = rospy.get_rostime()
    H = Header(stamp=Time, frame_id=ref_frame)
    wr = Wrench(force=self.array_to_Vector3(force_array), torque=self.array_to_Vector3(torque_array))
    msg = WrenchStamped(header=H,wrench=wr)
    return msg
  
  def check_overflow(self):
    if max(self.LLEG_force_array)>20:
      rospy.logwarn("LLEG_force_array overvlow")
      print(self.LLEG_count)
      print(self.LLEG_force_array)



  def publishZmp(self):
    while not rospy.is_shutdown():    

      # subscribe the force/torque sensor data
      self.LLEG_zmpsub = rospy.Subscriber("/robotv1/sensor/ft_sensor/LLEG_ft",WrenchStamped, self.LLEG_callback)
      self.RLEG_zmpsub = rospy.Subscriber("/robotv1/sensor/ft_sensor/RLEG_ft",WrenchStamped, self.RLEG_callback)
      
      # average the force/torque from the sensor
      self.ft_mean()

      # calculate the zmp point 
      LLEG_zmp_array = self.Cal_zmp_array(self.LLEG_force_array,self.LLEG_torque_array)
      RLEG_zmp_array = self.Cal_zmp_array(self.RLEG_force_array,self.RLEG_torque_array)
      
      # generate the ros message PointStamped
      LLEG_zmp_msg = self.zmp_msg(LLEG_zmp_array, self.LLEG_ft_sensor_frame)
      RLEG_zmp_msg = self.zmp_msg(RLEG_zmp_array, self.RLEG_ft_sensor_frame)
      
      # publish the zmp for single foot to the master
      self.LLEG_zmpPub.publish(LLEG_zmp_msg)
      self.RLEG_zmpPub.publish(RLEG_zmp_msg)
      
      # publish the zmp point transformation with respect of the sensor coordinates
      LLEG_zmp_frame_msg = self.zmp_frame_msg(LLEG_zmp_array, self.LLEG_zmp_frame, self.LLEG_ft_sensor_frame)
      RLEG_zmp_frame_msg = self.zmp_frame_msg(RLEG_zmp_array, self.RLEG_zmp_frame, self.RLEG_ft_sensor_frame)
 
      self.br.sendTransform(LLEG_zmp_frame_msg)
      self.br.sendTransform(RLEG_zmp_frame_msg)

      # generate the force/torque message with type  WrenchStamped in zmp coordinates
      LLEG_zmp_ft_msg = self.zmp_ft_msg(self.LLEG_force_array, self.LLEG_torque_array, self.LLEG_zmp_frame)
      RLEG_zmp_ft_msg = self.zmp_ft_msg(self.RLEG_force_array, self.RLEG_torque_array, self.RLEG_zmp_frame)
      
      # publish the zmp force/torque message on single foot to the master
      self.LLEG_zmp_ft_Pub.publish(LLEG_zmp_ft_msg)
      self.RLEG_zmp_ft_Pub.publish(RLEG_zmp_ft_msg)
      
      self.reset_arg()
      self.rate.sleep()
      
if __name__== "__main__":
  try:
    zmpNodePub = ZmpPub()
  
    zmpNodePub.publishZmp()
  except rospy.ROSInterruptException:
    pass

