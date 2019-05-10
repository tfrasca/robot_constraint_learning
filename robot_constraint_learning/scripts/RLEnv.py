import rospy
from rospy import ServiceException
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

import csv
import numpy as np
import matplotlib.pyplot as plt
#from lib import plotting

import Robbie
import Policy
from simple_dqn_agent import SimpleDQN
import time

class RLEnv:
  def __init__(self, stateSpace, actionSpace, nameSpace = "robotis_op3", robotType = None, seed=2, numEpisodes=10000, numSteps=500):

    #self.unpause = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
    np.random.seed(seed)
    self.robot = Robbie.Robbie(nameSpace)
    self.deleteModelService = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    self.currentModel = None
    self.fallen = False
    self.numEpisodes = numEpisodes
    self.numSteps = numSteps
    self.stateJointNames = stateSpace[0]
    self.stateJointStates = stateSpace[1]
    self.stateJointStepSize = stateSpace[2]
    self.actionJointNames = actionSpace[0]
    self.actionJointControl = actionSpace[1]
    self.numActions = len(self.actionJointControl[0])**len(self.actionJointNames)
    #print 'num actions:', self.numActions
    # change the policy type based on the state and action spaces
    #self.policy = Policy.Policy(stateSpace, actionSpace)
    self.epsilonMax =.5
    self.epsilonMin =0.05
    self.policy = SimpleDQN(self.numActions, 18, 7, .01, .99, .99, self.epsilonMax, 1)
    self.actions = []
    self.createActionSpace()
    self.rewards=[]
    self.episodeSteps =[]
    try:
      rospy.ServiceProxy("gazebo/unpause_physics",Empty).call()
    except ServiceException:
      pass

  def run(self, maxSteps=1000, explore = False):
      R = 0
      numSteps = 0
      while not self.fallen and numSteps < maxSteps:
        numSteps += 1
        initState = self.getState()
        #act = self.policy.selectAction(initState)
        act = self.policy.process_step(initState, explore)
        action = self.actions[act]
        #print "moving joints relative", action
        self.executeAction(action)
        newState = self.getState()
        reward = self.calcReward(newState, action)
        self.policy.give_reward(reward)
        R += reward
        #self.policy.updatePolicy(initState, newState, reward)
      return (numSteps, R)

  def explore(self):
    episode = 0
    print "exploring!"
    try:
      for episode in range(1,self.numEpisodes):
        t0 = time.time()
        while True:
          self.resetEnv()
          self.checkFallen()
          if not self.fallen:
            break
        print "episode: ", episode
        epsilon = self.epsilonMin + (self.epsilonMax - self.epsilonMin) * np.exp(episode * (np.log(0.01) / (self.numSteps * (3/4.0))))
        print "epsilon: ", epsilon
        self.policy.set_explore_epsilon(epsilon)
        if episode % 5 == 0:
          print "no exploration"
          steps,reward = self.run(explore = False)
          self.episodeSteps.append(steps)
          self.rewards.append(reward)
          print "steps:", steps
          print "reward:", reward
        else:
          self.run(explore = True)
        lapsedTime = time.time() - t0
        print "time", lapsedTime
        self.policy.finish_episode()
        self.policy.update_parameters()
    except KeyboardInterrupt as e:
      pass
    print self.rewards
    return self.policy

  def resetEnv(self):
    print "resetting env"
    self.robot.resetRobot()
    rospy.sleep(3)
    if self.currentModel:
      x = self.deleteModel(self.currentModel)
      self.currentModel = None
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
    self.checkFallen()
    if not self.fallen:
      position = self.robot.getPelvisPosition()
      reward = np.exp(1.0/(3*position.z))
      #print position.z
      dist = self.robot.getHandBoxDistance()
      reward += np.exp(1.0/(4*dist))
      return reward
    return -200

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

  def getState(self):
    state = self.robot.getState()
    newState = []
    name = state.name
    position = state.position
    velocity = state.velocity
    effort = state.effort
    for joint in self.stateJointNames:
      newState.extend([position[name.index(joint)], velocity[name.index(joint)], effort[name.index(joint)]])
    return np.array(newState)

  def createActionSpace(self):
    control = self.actionJointControl[0]
    for c0 in control:
      for c1 in control:
        for c2 in control:
          for c3 in control:
            for c4 in control:
              for c5 in control:
                self.actions.append([c0,c1,c2,c3,c4,c5])

  def saveData(self,rewards,numSteps):
    with open('data.csv', 'a') as csvfile:
        writer = csv.writer(csvfile, delimiter=",")
        writer.writerow(rewards)
        writer.writerow(numSteps)

if __name__ == "__main__":
  jointSpace = ["l_hip_pitch", "l_knee", "l_ank_pitch", "l_sho_pitch", "l_sho_roll", "l_el"]
  jointState = ["position","velocity","effort"]
  jointStateStepSize = None
  stateSpace = (jointSpace, jointState, jointStateStepSize)
  actionJoints = ["l_hip_pitch", "l_knee", "l_ank_pitch", "l_sho_pitch", "l_sho_roll", "l_el"]
  actionValues = 6*[[-0.015, 0.0, 0.015]]
  actionSpace = (actionJoints, actionValues)
  env = RLEnv(stateSpace, actionSpace)
  env.explore()
  rewards = env.rewards
  numSteps = env.episodeSteps
  env.saveData(rewards,numSteps)
