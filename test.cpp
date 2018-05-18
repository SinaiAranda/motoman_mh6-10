/*********************************************************************
 * CIDESI
 *********************************************************************/

/* Author: SINAÌ ARANDA MIRAMONTES */


#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <visualization_msgs/Marker.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);  

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("R1");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::planning_interface::MoveGroup::Plan my_plan;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id= "/base_link";
  line_strip.header.stamp= ros::Time::now();
  line_strip.ns= "line_strip";
  line_strip.action= visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w= 1.0;

  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.01;
  line_strip.scale.y = 0.01;
  line_strip.scale.z = 0.001;

  line_strip.color.r = 0.2;
  line_strip.color.g = 0.8;
  line_strip.color.b = 0.2;
  line_strip.color.a = 0.5;

  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.00525593757629 + 0.5;
  start_pose.position.y = -0.437265723944;
  start_pose.position.z = 0.398729532957;
  group.setStartStateToCurrentState();
  group.setPoseTarget(start_pose);
  group.plan(my_plan);

  group.execute(my_plan);
  group.clearPoseTarget();

  geometry_msgs::Point p;
  p.x = start_pose.position.x;
  p.y = start_pose.position.y;
  p.z = start_pose.position.z;

  geometry_msgs::Pose target_pose = start_pose;

  geometry_msgs::Pose earlier_target_pose = target_pose;

  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory;

  ROS_INFO("Pose X: %f, Pose Y: %f, Pose Z: %f", start_pose.position.x, start_pose.position.y, start_pose.position.z);

  float t, direction = 1.00;
  t = 0.5;

  for (float j = 1;j<=50;j++){
  
  for (float i = 1;i<=20;i++){

  ROS_INFO("i: %f", i);

  target_pose.position.x = group.getCurrentPose().pose.position.x - (0.001 * direction);
  p.x = target_pose.position.x;
  target_pose.position.y = group.getCurrentPose().pose.position.y - (0.001 * direction);
  p.y = target_pose.position.y;

  ROS_INFO("Pose X: %f, Pose Y: %f, Pose Z: %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

  line_strip.points.push_back(p);
  marker_pub.publish(line_strip);

  waypoints.push_back(target_pose);

  ROS_INFO("Compute Cartesian Path");
  group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  my_plan.trajectory_ = trajectory;
  sleep(t);

  ROS_INFO("Execute Plan");
  group.execute(my_plan);
  sleep(t);

  ROS_INFO("Sleep");
  waypoints.clear();
  sleep(t);
  }

  ROS_INFO("j: %f", j);

  target_pose.position.z = group.getCurrentPose().pose.position.z + 0.001;
  p.z = target_pose.position.z;

  ROS_INFO("Pose X: %f, Pose Y: %f, Pose Z: %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

  line_strip.points.clear();

  waypoints.push_back(target_pose);

  ROS_INFO("Compute Cartesian Path");
  group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  my_plan.trajectory_ = trajectory;
  sleep(t);

  ROS_INFO("Execute Plan");
  group.execute(my_plan);
  sleep(t);

  ROS_INFO("Sleep");
  waypoints.clear();
  sleep(t);

  direction = direction * -1;
  }

  ros::shutdown();  
  return 0;
}
