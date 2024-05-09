#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_robot_state");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();

  // Define the planning group that corresponds to the robot's move group
  static const std::string PLANNING_GROUP = "cr5_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Get the current state of the robot
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  // Get the number of variables (joint positions)
  size_t num_joints = current_state->getVariableCount();
  // Get the pointer to the joint values
  const double* joint_values_ptr = current_state->getVariablePositions();
  // Create a vector from the pointer
  std::vector<double> joint_values(joint_values_ptr, joint_values_ptr + num_joints);

  std::cout << "Current joint values of the robot:" << std::endl;
  for (double joint_value : joint_values) {
    std::cout << joint_value << " ";
  }
  std::cout << std::endl;

  // Optionally, print more details like position and orientation of the end effector
  const geometry_msgs::Pose& end_effector_state = move_group_interface.getCurrentPose().pose;
  std::cout << "End effector position (x, y, z): " 
            << end_effector_state.position.x << ", " 
            << end_effector_state.position.y << ", " 
            << end_effector_state.position.z << std::endl;
  std::cout << "End effector orientation (x, y, z, w): " 
            << end_effector_state.orientation.x << ", " 
            << end_effector_state.orientation.y << ", " 
            << end_effector_state.orientation.z << ", " 
            << end_effector_state.orientation.w << std::endl;

  ros::shutdown();
  return 0;
}
