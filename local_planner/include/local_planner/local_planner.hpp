#pragma once

// C++ Standard Libraries
#include "iostream"
#include "vector"
#include "cmath"
#include "memory"
#include "map"

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Custom Libraries
#include "multirobot_utils/structs.hpp"

class Robot : public rclcpp::Node
{
public:
  // ROS2 Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

  // ROS2 Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delayed_time_pub;

  // ROS2 Timer
  rclcpp::TimerBase::SharedPtr robot_timer;

  // Logical Variables
  bool initialized = false;

  // Character Variables
  std::string robot_name;

  // Numeric Variables
  double v_max = 0.6;
  double w_max = 2.5;
  double last_traj_received = 0.0;
  double last_traj_update = 0.0;
  double delayed_time = 0.0;
  double proj_ratio = 0.0;
  double traj_start_time = 0.0;

  // ROS2 Variables
  nav_msgs::msg::Path::SharedPtr traj_msg = std::make_shared<nav_msgs::msg::Path>();

  // Custom Variables
  Structs::Pose robot_pose;
  std::map<double, Structs::Pose> received_traj;
  std::map<double, Structs::Pose> updated_traj;

  // ROS2 Publisher/Subscriber Functions
  void set_pubs_subs();
  void get_traj(const nav_msgs::msg::Path::SharedPtr msg);
  void update_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publish_delayed_time();

  // Initialization Functions
  void set_params();

  // Update Functions
  void update_local_plan();
  void calculate_delayed_time();

  // Constructors
  Robot();

  // Loops
  void robot_loop();
};
