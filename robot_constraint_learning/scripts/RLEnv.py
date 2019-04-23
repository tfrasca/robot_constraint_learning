import rospy
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

import numpy as np
import matplotlib.pyplot as plt
#from lib import plotting

import Robbie
import Policy

class RLEnv:
  def __init__(self, stateSpace, actionSpace, nameSpace = "robotis_op3", robotType = None, seed=1, numEpisodes=5, numSteps=20):
    #self.unpause = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
    np.random.seed(seed)
    self.robot = Robbie.Robbie(nameSpace)
    self.deleteModelService = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    self.currentModel = None
    self.fallen = False
    self.numEpisodes = numEpisodes
    self.numSteps = numSteps
    self.actionJointNames = actionSpace[0]
    self.actionJointControl = actionSpace[1]
    # change the policy type based on the state and action spaces
    self.policy = Policy.Policy(stateSpace, actionSpace)
  
  def explore(self):
    try:
      for episode in range(self.numEpisodes):
        R = 0
        print "episode: ", episode
        for x in xrange(10):
          self.resetEnv()
          self.checkFallen()
          if not self.fallen:
            break
        numSteps = 0
        while not self.fallen and numSteps < self.numSteps:
          numSteps +=1
          print numSteps
          initState = self.robot.getState()
          act = self.policy.selectAction(initState)
          print "moving joints relative", act
          self.executeAction(act)
          newState = self.robot.getState()
          self.checkFallen()
          reward = self.calcReward(newState, act)
          R += reward
          self.policy.updatePolicy(initState, newState, reward)
    except KeyboardInterrupt as e:
      pass
    return self.policy

  def resetEnv(self):
    print "resetting env"
    self.robot.resetRobot()
    rospy.sleep(2)
    if self.currentModel:
      self.deleteModel(self.currentModel)
      self.currentModel = None
    #if np.random.randint(2):  # randomly spawn box weight
    if np.random.randint(1):  # spawn light box
      self.spawnModel("heavy_box")
    else:
      self.spawnModel("light_box")
    rospy.ServiceProxy("gazebo/reset_world",Empty).call()
    rospy.sleep(2)

  def spawnModel(self, modelName):
    model = rospy.get_param("light_box")
    if modelName == "heavy_box":
      model = rospy.get_param("heavy_box")
    initPose = Pose()
    initPose.position = Point(.166,0,.075)
    initPose.orientation = Quaternion(0,0,0,1)
    gazebo_interface.spawn_sdf_model_client(modelName, model, "", \
                                             initPose, "","/gazebo")
    self.currentModel = modelName

  def deleteModel(self, modelName):
    self.deleteModelService(modelName)

  def calcReward(self, state, action):
    if not self.fallen:
      position = self.robot.getPelvisPosition()
      return np.exp(2.0 / position.z)
    return -100

  def executeAction(self, action):
    state = self.robot.getState()
    jointNames = state.name
    jointPos = state.position
    for joint in jointNames:
      try:
        index = jointNames.index(joint)
        if joint in self.robot.getLegJoints():
          self.robot.setLegJoint(joint, jointPos[index])
        elif joint in self.robot.getArmJoints():
          self.robot.setArmJoint(joint, jointPos[index])
      except ValueError:
        pass
    self.robot.setRelativeSymetricLeg(self.actionJointNames[:3], action[:3])
    self.robot.setRelativeSymetricArm(self.actionJointNames[3:], action[3:])
    self.robot.move()

  def checkFallen(self):
    position = self.robot.getPelvisPosition()
    yThresh = 0.05
    zThresh = 0.1
    xThresh = .1
    if yThresh - np.abs(position.y) < 0 or \
       np.abs(position.z) - zThresh< 0 or \
       xThresh - np.abs(position.x) < 0:
      self.fallen=True
      print "I have fallen and i can't get up"
    else:
      self.fallen=False

if __name__ == "__main__":
  try:
    jointSpace = ["l_hip_pitch", "l_knee", "l_ank_pitch", "l_sho_pitch", "l_sho_roll", "l_el"]
    jointState = ["position","velocity","effort"]
    jointStateStepSize = None
    stateSpace = (jointSpace, jointState, jointStateStepSize)
    actionJoints = ["l_hip_pitch", "l_knee", "l_ank_pitch", "l_sho_pitch", "l_sho_roll", "l_el"]
    actionValues = 6*[[-0.015, 0.0, 0.015]]
    actionSpace = (actionJoints, actionValues)
    env = RLEnv(stateSpace, actionSpace)
    env.explore()
  except KeyboardInterrupt as e:
    print "quitting"
