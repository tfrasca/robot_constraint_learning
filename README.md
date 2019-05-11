# robot_constraint_learning
Env: ROS kinetic + Ubuntu 16.04


required packages

controllers:
velocity_controllers
effort_controllers
joint_state_controller
joint_state_publisher
joint_trajectory_action
joint_trajectory_controller

Robotis:
robotis_controller 
robotis_controller_msgs 
robotis_device 
robotis_framework_common 
robotis_manipulator 
robotis_math
robotis_control 
robotis_gazebo 
op3_action_edito
op3_action_module
op3_action_module_msgs
op3_balance_control
op3_base_module
op3_camera_setting_tool
op3_direct_control_module
op3_extra
op3_gazebo
op3_gui_demo
op3_head_control_module
op3_kinematics_dynamics
op3_localization
op3_manager
op3_offset_tuner_client
op3_offset_tuner_msgs
op3_offset_tuner_server
op3_online_walking_module

```catkin_make```

to run RL agent:
```roslaunch robot_constraint_learning robotis_sim.launch```

in new terminal:
```roscd robot_constraint_learning/scripts```

```python RLEnv.py```

*note!!!*

package `hardware` needs the real robot. 

package `robotv1_control`, `demo_moveit_config` need to put in a moveit workspace.

package `robotv1_gazebo`,`robotv1_description` need gazebo and rviz.

**For simulation of the robotv1(biped robot)**

Put package `robotv1_control`, `demo_moveit_config`, `robotv1_gazebo`,`robotv1_description` into a moveit workspace. 

Every launch file should be executed seperately, make sure only one of this launch file is running at one time.

for move pelvis by inputing w/s/a/d/R, one command at a time.run

```roslaunch robotv1_gazebo move_pelvis.launch```

(it's not teleop_keyboard, you need to press enter after typing a command). This launch file will start rviz and gazebo at the same time. 

for visualize the moveit motion plan through interactive marker, run

```roslaunch demo_moveit_config demo.launch```

for moveit plan execution on gazebo, run 

```roslaunch robotv1_gazebo robotv1_gazebo.launch```

```roslaunch demo_moveit_config moveit_planning_execution.launch ```

for visualize the robot in rviz with fake joint publisher, run 

```roslaunch robotv1_description robotv1_rviz.launch```

for start a gazebo simulation only, run 

```roslaunch robotv1_gazebo robotv1_gazebo.launch```


