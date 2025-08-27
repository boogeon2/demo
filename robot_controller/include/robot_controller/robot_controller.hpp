#pragma once

// C++ Standard Libraries
#include "string"
#include "memory"
#include "chrono"
#include "map"
#include "iostream"

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Custom Libraries
#include "multirobot_utils/structs.hpp"
#include "rmse_logger.hpp"

enum class ControlMode
{
  IDLE,
  FORWARD,
  BACKWARD,
  ROTATE_IN_PLACE
};

class RobotController : public rclcpp::Node
{
public:
  // ROS2 Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr new_robot_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

  // ROS2 Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_progress_pub;

  // ROS2 Timer
  rclcpp::TimerBase::SharedPtr robot_timer;

  // Logger
  std::unique_ptr<Logger> logger_;
  nav_msgs::msg::Path trajectory_;
  geometry_msgs::msg::PoseStamped current_pose_;

  // Logical Variables
  bool rotation_finished = true;
  bool is_first_rotation_step = true;

  // Character Variables
  std::string robot_name;
  std::string log_file_path;

  // Numeric Variables
  double lin_vel = 0.6;
  double ang_vel = 2.5;
  double min_lin_vel = 0.5;
  double max_lin_vel = 0.7;
  double regulated_lin_vel = 0.0;
  double regulated_ang_vel = 0.0;
  double lookahead_dist = 0.4;
  double distance_difference = 0.0;
  double heading_difference = 0.0;
  double path_difference = 0.0;
  double prev_heading_difference = 0.0;
  double last_traj_update = 0.0;
  double prev_arrival_time = 0.0;
  double real_time_factor = 1.0;
  size_t current_waypoint_idx = 0;
  std::vector<double> hold_times;

  // Custom Variables
  Structs::Pose robot_pose;
  Structs::Velocity cmd_vel;
  std::vector<Structs::Pose> waypoints;
  ControlMode current_mode = ControlMode::IDLE;

  // ROS2 Publisher/Subscriber Functions
  void set_pubs_subs();
  void register_robot(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void update_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void get_traj(const nav_msgs::msg::Path::SharedPtr msg);
  void publish_cmd_vel();
  void publish_traj_progress();

  // Helper Functions
  double normalize_angle_diff(double angle, double angle2);

  // Initialization Functions
  void set_params();
  void init_logger();

  // Update Functions
  void determine_current_waypoint();
  void determine_mode();
  void calculate_cmd_vel();
  void update_logger();

  // Constructors
  RobotController();

  // Loops
  void robot_controller_loop();
};
