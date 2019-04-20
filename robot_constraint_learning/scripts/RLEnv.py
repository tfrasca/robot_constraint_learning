import rospy
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty
import Robbie
import numpy as np

class RLEnv:
  def __init__(self, robotType, nameSpace = "robotis_op3", seed=1):
    np.random.seed(seed)
    self.robot = Robbie.Robbie(nameSpace)
    #self.node = rospy.init_node("rl_env_setup")
    self.deleteModel = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    #self.unpause = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
  
  def explore(self):
    for episode in range(self.numEpisodes):
      self.resetEnv()
      while not self.isTerminated():
        act = self.selectAction()
        self.executeAction(act)
        self.updatePolicy()
    # return policy
    return None

  def resetEnv(self):
    self.robot.resetRobot()
    rospy.sleep(5)
    self.deleteModel(self.currentModel)
    rospy.ServiceProxy("gazebo/reset_world",Empty).call()
    #if np.random.randint(2):  # randomly spawn box weight
    if np.random.randint(1):  # spawn light box
      self.spawnModel("light_box")
    else:
      self.spawnModel("heavy_box")

  def spawnModel(self, modelName):
    try:
      model = rospy.get_param("light_box")
      if modelName == "heavy_box":
        model = rospy.get_param("heavy_box")
      initPose = Pose()
      initPose.position = Point(.166,0,.1)
      initPose.orientation = Quaternion(0,0,0,1)
      gazebo_interface.spawn_urdf_model_client(modelName, model, "", \
                                               initPose, "","/gazebo")
    except e:
      print e

  def deleteModel(self,modelName):
    self.deleteModel(modelName)

  #def updateModel(self, initialState, finalState, action=None, reward):
  #  pre = np.expand_dims(np.append(initialState, 0.0), 1)
  #  post = np.expand_dims(np.append(finalState, 0.0), 1)
  #  v = r + self.gamma * np.dot(self.theta.T, post) - np.dot(self.theta.t,pre)
  #  delta = np.abs(v)
  #  self.theta += self.alpha * delta * pre

  def selectAction(self, state):
    vsdfasdf = np.zeros((self.actionCount,))
    for ap in range(self.actionCount):
      pre = np.expand_dims(np.append(state, 0.0), 1)
      v[ap] = np.dot(self.theta.T, pre)
    a = 0 if v[0] > 0 else 1
    return a

  def isTerminated():
    return True

    
if __name__ == "__main__":
  env = RLEnv()
  env.explore()
