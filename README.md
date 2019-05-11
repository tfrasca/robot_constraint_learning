# robot_constraint_learning

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
