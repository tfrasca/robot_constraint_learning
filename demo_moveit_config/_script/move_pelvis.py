#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MovePelvis(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_pelvis',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    lleg_group = moveit_commander.MoveGroupCommander("LLEG")
    rleg_group = moveit_commander.MoveGroupCommander("RLEG")
    legs_group = moveit_commander.MoveGroupCommander("LEGs")

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    planning_frame = legs_group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.legs_group = legs_group
    self.lleg_group = lleg_group
    self.rleg_group = rleg_group

    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame

  def go_to_pose_goal(self,move_x=0.0,move_z=0.0):
    # Copy class variables to local variables to make the web move_pelviss more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    lleg_goal_pose = geometry_msgs.msg.Pose()
    rleg_goal_pose = geometry_msgs.msg.Pose()
    if True:
      lleg_goal_pose = copy.deepcopy(self.lleg_group.get_current_pose().pose)
      rleg_goal_pose = copy.deepcopy(self.rleg_group.get_current_pose().pose)
      ## BEGIN_SUB_TUTORIAL plan_to_pose
      ##
      ## Planning to a Pose Goal
      ## ^^^^^^^^^^^^^^^^^^^^^^^
      ## We can plan a motion for this group to a desired pose for the
      ## end-effector:
      lleg_goal_pose.position.x -= move_x
      lleg_goal_pose.position.z -= move_z
      rleg_goal_pose.position.x -= move_x
      rleg_goal_pose.position.z -= move_z
      print(lleg_goal_pose)
      raw_input()
      
      self.lleg_group.set_pose_target(pose_to_list(lleg_goal_pose))
      self.rleg_group.set_pose_target(pose_to_list(rleg_goal_pose))
      ## Now, we call the planner to compute the plan and execute it.
      self.legs_group.go(wait=True)
      # Calling `stop()` ensures that there is no residual movement
      self.legs_group.stop()
      # It is always good to clear your targets after planning with poses.
      # Note: there is no equivalent function for clear_joint_value_targets()
      self.legs_group.clear_pose_targets()

# 
#   def display_trajectory(self, plan):
#     robot = self.robot
#     display_trajectory_publisher = self.display_trajectory_publisher
# 
#     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#     display_trajectory.trajectory_start = robot.get_current_state()
#     display_trajectory.trajectory.append(plan)
#     
#     # Publish
#     display_trajectory_publisher.publish(display_trajectory);
# 
#   def execute_plan(self, plan):
#     self.legs_group.execute(plan, wait=True)
# 

def main():
  try:
    print "============ Press `Enter` to begin the tet by setting up the moveit_commander (press ctrl-d to exit) ..."
 #   raw_input()
    move_pelvis = MovePelvis()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    x = raw_input("move_x = ")
    z = raw_input("move_z = ")

    move_pelvis.go_to_pose_goal(move_x = float(x),move_z = float(z))
   
    print "============ Python move_pelvis demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

