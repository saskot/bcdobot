#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


void positionCallback(const geometry_msgs::Point::ConstPtr& msg, moveit::planning_interface::MoveGroupInterface* move_group_interface) {

  geometry_msgs::Pose target_pose;
  target_pose.position.x = msg->x;
  target_pose.position.y = msg->y;
  target_pose.position.z = msg->z + 0.1; //  10 cm

 
  move_group_interface->setPoseTarget(target_pose);
  move_group_interface->move();

 
  target_pose.position.z = msg->z; 
  move_group_interface->setPoseTarget(target_pose);
  move_group_interface->move();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "posli_ruku_niekde");
  ros::NodeHandle node_handle;


  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  static const std::string PLANNING_GROUP = "cr5_new";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Subscribe to the object_pose topic
  ros::Subscriber point_sub = node_handle.subscribe<geometry_msgs::Point>("object_pose", 1, boost::bind(positionCallback, _1, &move_group_interface));


  ros::waitForShutdown();

  return 0;
}