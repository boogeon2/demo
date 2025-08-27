#pragma once

// C++ Standard Libraries
#include "string"
#include "memory"
#include "chrono"
#include "cmath"
#include "limits"

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

// External Libraries
#include "Eigen/Dense"

// Custom Libraries
#include "multirobot_utils/structs.hpp"

class FakeRobot : public rclcpp::Node
{
public:
  // ROS2 Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

  // ROS2 Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // ROS2 Timer
  rclcpp::TimerBase::SharedPtr fake_robot_timer;

  // Logical Variables
  bool initialized = false;

  // Character Variables
  std::string robot_name;

  // Numeric Variables
  double update_rate = 50.0; // Hz
  double timeout = 0.5;      // seconds

  // ROS2 Variables
  geometry_msgs::msg::Twist current_cmd_vel;
  rclcpp::Time last_cmd_vel_time;

  // Custom Variables
  Structs::Pose robot_pose;

  // ROS2 Subscriber Functions
  void set_pubs_subs();
  void register_robot(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void get_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish_pose();
  void publish_tf();

  // Initialization Functions
  void set_params();

  // Update Functions
  void update_pose();

  // Constructors
  FakeRobot();

  // Loops
  void fake_robot_loop();
};
