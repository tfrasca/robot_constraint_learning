import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPub:
  def __init__(self,**kwargs):
    self.nn = rospy.init_node("trajectoryPublisher")
    self.trajPub = rospy.Publisher("/robotv1/joint_trajectory_controller/command", JointTrajectory, queue_size=1)

    self.jointNames = ["leftHipJoint","leftKneeJoint"]

  def publishTrajectory(self, trajectoryPositions, jointNames=None):
    if jointNames == None:
      jointNames = self.jointNames
    trajPoses = []
    for x in xrange(5):
      trajPoses.append(JointTrajectoryPoint(positions=[-.3+0.1*x,.0],time_from_start=rospy.Duration.from_sec(x*.1), effort=[100,10]))

    #trajMsg = JointTrajectory(joint_names = jointNames, points = trajPoses)
    trajmsg = JointTrajectory(joint_names=["leftHipJoint", "leftKneeJoint"], points = trajPoses)
    rospy.sleep(5)
    self.trajPub.publish(trajmsg)
    rospy.sleep(5)
      
if __name__ == "__main__":
  positions = [[0.4,0.4]]
  trajNodePub = TrajectoryPub()
  trajNodePub.publishTrajectory(positions)

  rospy.spin()
