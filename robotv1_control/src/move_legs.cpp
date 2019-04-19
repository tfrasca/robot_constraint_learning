 //#include "ros/ros.h"
 //#include "std_msgs/String.h"
 //#include <trajectory_msgs/JointTrajectoryPoint.h>
 //#include <trajectory_msgs/JointTrajectory.h>
 //
 //#include <sstream>
 //int main(int argc, char **argv)
 //{
 //  ros::init(argc, argv, "talker");
 //  ros::NodeHandle n;
 //  ros::Publisher chatter_pub = n.advertise<trajectory_msgs::JointTrajectoryPoint>("chatter", 1000);
 //  ros::Rate loop_rate(10);
 //  int count = 0;
 //  while (ros::ok())
 //  {
 //    std_msgs::String msg;
 //
 //    std::stringstream ss;
 //    ss << "hello world " << count;
 //    msg.data = ss.str();
 //    ROS_INFO("%s", msg.data.c_str());
 //    
 //    trajectory_msgs::JointTrajectoryPoint traj_waypoints;
 //    for(std::size_t i=0; i<6; i++){
 //        traj_waypoints.positions.push_back(0.0);
 //    }
 //
 //    ROS_INFO_STREAM("print\n" << traj_waypoints << "\n");
 //    chatter_pub.publish(traj_waypoints);
 //    ros::spinOnce();
 //    loop_rate.sleep();
 //    ++count;
 //  }
 //
 //
 //  return 0;
 //}
 //
 //
#include <ros/ros.h>
#include <sstream>
#include <iostream>
// Ros messages
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

// MoveIt! IK solve 
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// MoveIt! move group interface
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    // initialize node 
    ros::init(argc, argv, "move_legs");
    // node objedt node_handle
    ros::NodeHandle nh;
    ros::Publisher lleg_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/robotv1/LLEG_controller/command", 10);
    ros::Publisher rleg_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/robotv1/RLEG_controller/command", 10);
    //ros::Rate r(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // initialize moveit planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // initialize moveit group 
    static const std::string LEGs_GROUP = "LEGs";
    static const std::string LLEG_GROUP = "LLEG";
    static const std::string RLEG_GROUP = "RLEG";
    moveit::planning_interface::MoveGroupInterface legs_move_group(LEGs_GROUP);
    moveit::planning_interface::MoveGroupInterface lleg_move_group(LLEG_GROUP);
    moveit::planning_interface::MoveGroupInterface rleg_move_group(RLEG_GROUP);
    // get current robot state
    const robot_state::JointModelGroup* legs_group = legs_move_group.getCurrentState()->getJointModelGroup(LEGs_GROUP);
    const robot_state::JointModelGroup* lleg_group = legs_move_group.getCurrentState()->getJointModelGroup(LLEG_GROUP);
    const robot_state::JointModelGroup* rleg_group = legs_move_group.getCurrentState()->getJointModelGroup(RLEG_GROUP);
    
    legs_move_group.allowReplanning(true);
    // initialize names of joint group
    const std::vector<std::string>& lleg_joint_names = lleg_group->getVariableNames();
    const std::vector<std::string>& rleg_joint_names = rleg_group->getVariableNames();
    const std::vector<std::string>& legs_joint_names = legs_group->getVariableNames();
       
    // initialize current joint positions 
    std::vector<double> lleg_joint_values;
    std::vector<double> rleg_joint_values;
    std::vector<double> legs_joint_values;
    
    double z_step = 0.03;
    double x_step = 0.03;
    float time_step = 2;

    
    while (ros::ok()){
        // get current robot state
        moveit::core::RobotStatePtr current_state = legs_move_group.getCurrentState();
        
        // get current robot joint values
        // current_state->copyJointGroupPositions(lleg_group, lleg_joint_values);
        // current_state->copyJointGroupPositions(rleg_group, rleg_joint_values);
        //current_state->copyJointGroupPositions(legs_group, legs_joint_values);
        // set the current time to the header of the trajectory msg
    
        
        // get current eef pose 
        const Eigen::Affine3d& lleg_eef_state = current_state->getGlobalLinkTransform("LLEG_l2");
        const Eigen::Affine3d& rleg_eef_state = current_state->getGlobalLinkTransform("RLEG_l2");
        // set goal pose  
        Eigen::Affine3d ltest = Eigen::Affine3d::Identity();
        ltest.translation() = lleg_eef_state.translation();
      
        Eigen::Affine3d rtest = Eigen::Affine3d::Identity();
        rtest.translation() = rleg_eef_state.translation();
        
        ROS_INFO("Input 'w'/'s' for +/-z,'a'/'d' for +/-x, 'R' for reset to AllZero Joint");
        std::string str;
        std::cin>>str;
        bool dol = false;
        if (str=="w"){
            ltest.translation().z() -= z_step;
            rtest.translation().z() -= z_step;
            dol = true;
        }
        if (str=="s"){
            ltest.translation().z() += z_step;
            rtest.translation().z() += z_step;
            dol = true;
        }
        if (str=="a"){
            ltest.translation().x() -= x_step;
            rtest.translation().x() -= x_step;
            dol = true;
        }
        if (str=="d"){
            ltest.translation().x() += x_step;
            rtest.translation().x() += x_step;
            dol = true;
        }
        if ((ltest.translation().z() > 0.20 )||(ltest.translation().z() < z_step + 0.01)){
            dol = false;
        }
        if (str=="R"){
            for(std::size_t i = 0; i<legs_joint_names.size(); ++i){
                legs_joint_values.insert(legs_joint_values.begin()+i,0.0);
            }
            legs_move_group.setJointValueTarget(legs_joint_values);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      
            bool success = (legs_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Resting 1 (pose goal) %s", success ? "" : "FAILED");
            legs_move_group.move();
            dol = false;
        }

        if (dol){
          if (ltest.translation().z() < 100){
              // solve IK 
              std::size_t attempts = 10;
              double timeout = 0.1;
              bool found_ik_lleg = current_state->setFromIK(lleg_group, ltest, attempts, timeout);
              bool found_ik_rleg = current_state->setFromIK(rleg_group, rtest, attempts, timeout);
              current_state->enforceBounds();
       
              trajectory_msgs::JointTrajectory lleg_traj;
              lleg_traj.header.stamp = ros::Time::now();
              
              trajectory_msgs::JointTrajectory rleg_traj;
              rleg_traj.header.stamp = ros::Time::now();
             
              trajectory_msgs::JointTrajectoryPoint lleg_traj_waypoint;
              lleg_traj_waypoint.time_from_start = ros::Duration(time_step);
              
              trajectory_msgs::JointTrajectoryPoint rleg_traj_waypoint;
              rleg_traj_waypoint.time_from_start = ros::Duration(time_step);
            
              // Now, we can print out the IK solution (if found):
              if (found_ik_lleg && found_ik_rleg){
                  current_state->copyJointGroupPositions(lleg_group, lleg_joint_values);
                  current_state->copyJointGroupPositions(rleg_group, rleg_joint_values);
                  for (std::size_t i = 0; i < lleg_joint_names.size(); ++i){
                     lleg_traj.joint_names.push_back(lleg_joint_names[i]);
                     rleg_traj.joint_names.push_back(rleg_joint_names[i]);
                  
                     lleg_traj_waypoint.positions.push_back(lleg_joint_values[i]);
                     lleg_traj_waypoint.effort.push_back(10.0);
                     
                     rleg_traj_waypoint.positions.push_back(rleg_joint_values[i]);
                     rleg_traj_waypoint.effort.push_back(10.0);
             

                  }
                  rleg_traj.points.push_back(lleg_traj_waypoint);
                  lleg_traj.points.push_back(rleg_traj_waypoint);

                  lleg_pub.publish(lleg_traj);
                  rleg_pub.publish(rleg_traj);
              }
              else{
                  ROS_INFO("Did not find IK solution, perhaps outside the work space");
                  ROS_INFO_STREAM("the eef transform is\n " << ltest.translation() << "\n");
              }
 
          }
          else{
              ROS_INFO("Reach the height limit\n");
          }
        }
        else{
            ROS_INFO("Input the right instruction! \n");
        }
        spinner.start();
    }
    ros::shutdown;
    return 0;
  
}
