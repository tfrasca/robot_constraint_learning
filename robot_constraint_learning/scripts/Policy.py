import numpy as np

class Policy:
  def __init__(self, stateSpace, actionSpace):
    self.stateSpace = stateSpace
    self.actionSpace = actionSpace
    self.epsilon = 1.0
    self.gamma = 0.99
    self.bias = 0.1
    self.alpha = 1e-6
    self.stateJointNames = stateSpace[0]
    self.stateJointStates = stateSpace[1]
    self.stateJointStepSize = stateSpace[2]
    self.actionJointNames = actionSpace[0]
    self.actionJointControl = actionSpace[1]
    self.stateCount = 0
    self.createStateSpace()
    self.theta = np.zeros((self.stateCount +1, 1))
    self.theta = np.random.normal(loc=0.0, scale=0.01, size=(self.stateCount+1, 1))

  def updatePolicy(self, initialState, finalState, reward, action=None):
    startState = self.extractState(initialState)
    endState = self.extractState(finalState)
    pre = np.expand_dims(np.append(startState, 0.0), 1)
    post = np.expand_dims(np.append(endState, 0.0), 1)
    v = reward + self.gamma * np.dot(self.theta.T, post) - np.dot(self.theta.T,pre)
    delta = np.abs(v)
    self.theta += self.alpha * delta * pre

  def selectAction(self, state):
    a = []
    # randomly select relative position change -.01, 0, 0.01
    if np.random.random() < self.epsilon:
      return 
    else: # select action with highest value
      v = np.zeros((self.actionCount,))
      for ap in range(self.actionCount):
        pre = np.expand_dims(np.append(state, 0.0), 1)
        v[ap] = np.dot(self.theta.T, pre)
      a = 0 if v[0] > 0 else 1
    return a

  def isTerminated(self):
    # if pelvis outside threshold x,y,z
    #   or max time step
    #   return true
    # else
    #   false
    return False

  def extractState(self, state):
    newState = []
    name = state.name
    position = state.position
    velocity = state.velocity
    effort = state.effort
    for joint in self.stateJointNames:
      newState.extend([position[name.index(joint)], velocity[name.index(joint)], effort[name.index(joint)]])
    return newState

  def createStateSpace(self):
    if self.stateJointStepSize:
      #createTabular
      pass
    else:
      self.stateCount = len(self.stateJointNames) * len(self.stateJointStates)
