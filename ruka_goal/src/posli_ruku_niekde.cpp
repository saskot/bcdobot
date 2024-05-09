#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "pcl_ros/transforms.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "posli_ruku_niekde");
  ros::NodeHandle node_handle;
  // Start an asynchronous spinner to allow MoveIt to process callbacks
  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate rate(10.0);



  // Define the planning group
  static const std::string PLANNING_GROUP = "cr5_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Set the planner and planning time
  move_group_interface.setPlannerId("RRTConnectConfigDefault");
  move_group_interface.setPlanningTime(5.0);
  move_group_interface.setMaxVelocityScalingFactor(0.05);  // Set speed to 50% of maximum
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  std::vector<double> home_position = {0.0875245, -0.439538, 1.94581, 1.6827, 1.69706, 0.036351};

  while (ros::ok()) {
    try {

      geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("camera_frame", "objekt", ros::Time(0));
      
      moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
      move_group_interface.setStartState(*current_state);


        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("camera_frame", "world",
                                  ros::Time(0));
      // Define the poses based on the transform
      geometry_msgs::Pose above_target_pose;
      above_target_pose.position.x = transformStamped.transform.translation.x;
      above_target_pose.position.y = transformStamped.transform.translation.y;
      above_target_pose.position.z = transformStamped.transform.translation.z + 0.15;  // 15 cm above the target
      above_target_pose.orientation = transformStamped.transform.rotation;

      geometry_msgs::Pose target_pose;
      target_pose.position.x = transformStamped.transform.translation.x;
      target_pose.position.y = transformStamped.transform.translation.y;
      target_pose.position.z = transformStamped.transform.translation.z;
      target_pose.orientation = transformStamped.transform.rotation;

      // Move to the position above the target
      move_group_interface.setPoseTarget(above_target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan_to_above;
      bool success_to_above = (move_group_interface.plan(plan_to_above) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success_to_above) {
        move_group_interface.move();
      } else {
        ROS_WARN("Failed to plan the motion to the position above the target");
        continue;
      }

      // Then move to the target position
      move_group_interface.setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan_to_target;
      bool success_to_target = (move_group_interface.plan(plan_to_target) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success_to_target) {
        ros::Duration(10).sleep();
        move_group_interface.move();
      } else {
        ROS_WARN("Failed to plan the motion to the target position");
      }

      // Then move to the home position
      move_group_interface.setJointValueTarget(home_position);
      moveit::planning_interface::MoveGroupInterface::Plan plan_to_home;
      bool success_to_home = (move_group_interface.plan(plan_to_home) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success_to_home) {
        ros::Duration(2).sleep();
        move_group_interface.move();
      } else {
        ROS_WARN("Failed to plan the motion to the home position");
      }

      }catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();  // Wait a bit before trying again
    }
   rate.sleep();
 }

  ros::shutdown();
  return 0;
}
