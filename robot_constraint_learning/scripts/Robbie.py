import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetLinkState

import numpy as np

class Robbie:

  def __init__(self, nameSpace='robotis_op3', *kwargs):
    self.node = rospy.init_node('robbie_' + nameSpace)
    self.nameSpace = '/' + nameSpace
    self.setupPubSubs()
    self.setupServices()
    self.setupJoints()
    self.actionNameToRL = {}
    self.jointState = None
    self.linkStates = None
    return

  def setupPubSubs(self):
    self.legTrajPub = rospy.Publisher(self.nameSpace + '/leg_traj_controller/command', JointTrajectory, queue_size=1)
    self.armTrajPub = rospy.Publisher(self.nameSpace + '/arm_traj_controller/command', JointTrajectory, queue_size=1)
    self.jointStateSub = rospy.Subscriber(self.nameSpace + '/joint_states', JointState, self.jointStateCallback)
    self.gazeboLinkSub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCallback)

  def setupServices(self):
    self.gazeboLinkSrv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

  def setupJoints(self):
    self.legJoints = [
     'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ank_pitch', 'l_ank_roll', 'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ank_roll', 'r_ank_pitch']
    self.legPositions = 12 * [0.0]
    self.legVelocities = 12 * [0.0]
    self.legEfforts = 12 * [10]
    self.armJoints = ['l_sho_roll', 'l_sho_pitch', 'l_el', 'r_sho_roll', 'r_sho_pitch', 'r_el']
    self.armPositions = 6 * [0.0]
    self.armVelocities = 6 * [0.0]
    self.armEfforts = 6 * [10]

  def setLegJoint(self, jointName, pos=None):
    if pos is not None:
      index = self.legJoints.index(jointName)
      self.legPositions[index] = pos
    return

  def setArmJoint(self, jointName, pos=None):
    if pos is not None:
      index = self.armJoints.index(jointName)
      self.armPositions[index] = pos
    return

  def setSymetricLeg(self, jointNames, positions):
    try:
      for joint, position in zip(jointNames, positions):
        if joint == 'l_hip_pitch':
          self.setLegJoint(joint, -1 * position)
          self.setLegJoint('r_hip_pitch', position)
        elif joint == 'l_knee':
          self.setLegJoint(joint, position)
          self.setLegJoint('r_knee', -1 * position)
        elif joint == 'l_ank_pitch':
          self.setLegJoint(joint, position)
          self.setLegJoint('r_ank_pitch', -1 * position)

    except Exception as e:
      print e

  def setRelativeSymetricLeg(self, jointNames, positions):
    try:
      currentPositions = self.jointState.position
      for joint, position in zip(jointNames, positions):
        position += currentPositions[jointNames.index(joint)]
        if joint == 'l_hip_pitch':
          self.setLegJoint(joint, -1 * position)
          self.setLegJoint('r_hip_pitch', position)
        elif joint == 'l_knee':
          self.setLegJoint(joint, position)
          self.setLegJoint('r_knee', -1 * position)
        elif joint == 'l_ank_pitch':
          self.setLegJoint(joint, position)
          self.setLegJoint('r_ank_pitch', -1 * position)

    except Exception as e:
      print e

  def setSymetricArm(self, jointNames, positions):
    try:
      for joint, position in zip(jointNames, positions):
        if joint == 'l_sho_pitch':
          self.setArmJoint(joint, -1 * position)
          self.setArmJoint('r_sho_pitch', position)
        elif joint == 'l_sho_roll':
          self.setArmJoint(joint, position)
          self.setArmJoint('r_sho_roll', -1 * position)
        elif joint == 'l_el':
          self.setArmJoint(joint, -1 * position)
          self.setArmJoint('r_el', position)

    except Exception as e:
      print e

  def setRelativeSymetricArm(self, jointNames, positions):
    try:
      currentPositions = self.jointState.position
      jointNames = self.jointState.name
      for joint, position in zip(jointNames, positions):
        position += currentPositions[jointNames.index(joint)]
        if joint == 'l_sho_pitch':
          self.setArmJoint(joint, -1 * position)
          self.setArmJoint('r_sho_pitch', position)
        elif joint == 'l_sho_roll':
          self.setArmJoint(joint, position)
          self.setArmJoint('r_sho_roll', -1 * position)
        elif joint == 'l_el':
          self.setArmJoint(joint, -1 * position)
          self.setArmJoint('r_el', position)

    except Exception as e:
      print e

  def resetRobot(self):
    self.legPositions = 12 * [0.0]
    self.armPositions = 6 * [0.0]
    self.move()

  def setEffort(self, jointName, effort):
    index = self.legJoints.index(jointName)
    self.legEfforts[index] = effort

  def moveLeg(self):
    trajPoint = JointTrajectoryPoint(positions=self.legPositions, effort=self.legEfforts, time_from_start=rospy.Duration.from_sec(1))
    trajMsg = JointTrajectory(joint_names=self.legJoints, points=[trajPoint])
    self.legTrajPub.publish(trajMsg)

  def moveArm(self):
    trajPoint = JointTrajectoryPoint(positions=self.armPositions, effort=self.armEfforts, time_from_start=rospy.Duration.from_sec(1))
    trajMsg = JointTrajectory(joint_names=self.armJoints, points=[trajPoint])
    self.armTrajPub.publish(trajMsg)

  def move(self):
    self.moveLeg()
    self.moveArm()
    rospy.sleep(2)

  def doAction(self):
    policy = self.actionNameToRL(actionName)
    while not policy.isTerminated():
      act = policy.getStep(self.getState())

  def jointStateCallback(self, jointState):
    self.jointState = jointState

  def getState(self):
    return self.jointState

  def getLegJoints(self):
    return self.legJoints

  def getArmJoints(self):
    return self.armJoints

  def linkStatesCallback(self, linkStates):
    self.linkStates = linkStates
    states = linkStates

  def getPelvisPosition(self):
    names = self.linkStates.name
    position = self.linkStates.pose[names.index('robotis::body_link')].position
    return position

  def getHandBoxDistance(self):
    try:
      linkState = self.gazeboLinkSrv.call("l_el_link","light_box")
      x = linkState.link_state.pose.position.x
      y = linkState.link_state.pose.position.y - (.125/2)
      z = linkState.link_state.pose.position.z 
      return np.sqrt(x**2 + y**2 + z**2)
    except Exception:
      return 0
