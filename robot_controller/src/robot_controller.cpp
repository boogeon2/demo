#include "robot_controller/robot_controller.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

void RobotController::set_pubs_subs()
{
  traj_sub = this->create_subscription<nav_msgs::msg::Path>(
      "traj", 10, std::bind(&RobotController::get_traj, this, std::placeholders::_1));

  pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", 10, std::bind(&RobotController::update_pose, this, std::placeholders::_1));

  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

  traj_progress_pub = this->create_publisher<nav_msgs::msg::Path>(
      "traj_progress", 100);

  robot_timer = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&RobotController::robot_controller_loop, this));
}

void RobotController::update_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  robot_pose.position = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

  tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
  q.normalize();

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose.orientation = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0);

  // Store current pose for logger
  current_pose_ = *msg;

  // Update logger with current pose if trajectory is available
  if (logger_ && !trajectory_.poses.empty())
  {
    update_logger();
  }
}

void RobotController::get_traj(const nav_msgs::msg::Path::SharedPtr msg)
{
  // Log the completion of previous trajectory if it exists
  if (logger_ && !trajectory_.poses.empty())
  {
    RCLCPP_INFO(this->get_logger(), "New trajectory received. Finalizing logging for previous trajectory.");
    update_logger();
  }

  hold_times.clear();
  waypoints.clear();
  current_waypoint_idx = 0;

  double start_time = msg->poses[0].header.stamp.sec + msg->poses[0].header.stamp.nanosec * 1e-9;

  for (size_t i = 0; i < msg->poses.size(); i++)
  {
    double timestamp = msg->poses[i].header.stamp.sec + msg->poses[i].header.stamp.nanosec * 1e-9;
    double relative_time = timestamp - start_time;

    Eigen::Vector2d position(
        msg->poses[i].pose.position.x,
        msg->poses[i].pose.position.y);

    tf2::Quaternion q(
        msg->poses[i].pose.orientation.x,
        msg->poses[i].pose.orientation.y,
        msg->poses[i].pose.orientation.z,
        msg->poses[i].pose.orientation.w);
    q.normalize();

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Structs::Pose pose(position.x(), position.y(), yaw);
    hold_times.push_back(relative_time);
    waypoints.push_back(pose);
  }

  last_traj_update = this->now().seconds();
  real_time_factor = 1.0;
  RCLCPP_INFO(this->get_logger(), "경로 수신 완료: %zu 개의 포인트", waypoints.size());

  // Store the trajectory for logger
  trajectory_ = *msg;

  // Reset any logger state if needed for the new trajectory
  if (logger_)
  {
    RCLCPP_INFO(this->get_logger(), "Starting logging for new trajectory");
  }
}

void RobotController::publish_cmd_vel()
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = cmd_vel.linear;
  twist_msg.angular.z = cmd_vel.angular;
  cmd_vel_pub->publish(twist_msg);
}

void RobotController::publish_traj_progress()
{
  nav_msgs::msg::Path progress_msg;
  progress_msg.header.stamp = this->now();
  progress_msg.header.frame_id = "map";
  for (size_t i = current_waypoint_idx; i < waypoints.size(); i++)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = waypoints[i].position(0);
    pose_stamped.pose.position.y = waypoints[i].position(1);
    pose_stamped.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, waypoints[i].orientation(2));
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    progress_msg.poses.push_back(pose_stamped);
  }
  traj_progress_pub->publish(progress_msg);
}

double RobotController::normalize_angle_diff(double angle, double angle2)
{
  double angDiff = angle - angle2;
  while (angDiff > M_PI)
    angDiff -= 2.0 * M_PI;
  while (angDiff <= -M_PI)
    angDiff += 2.0 * M_PI;
  return angDiff;
}

void RobotController::set_params()
{
  declare_parameter("robot_name", robot_name);
  declare_parameter("lin_vel", lin_vel);
  declare_parameter("ang_vel", ang_vel);
  declare_parameter("min_lin_vel", min_lin_vel);
  declare_parameter("max_lin_vel", max_lin_vel);
  declare_parameter("lookahead_dist", lookahead_dist);
  declare_parameter("log_file_path", "/logs/default.yaml");

  get_parameter("robot_name", robot_name);
  get_parameter("lin_vel", lin_vel);
  get_parameter("ang_vel", ang_vel);
  get_parameter("min_lin_vel", min_lin_vel);
  get_parameter("max_lin_vel", max_lin_vel);
  get_parameter("lookahead_dist", lookahead_dist);
  get_parameter("log_file_path", log_file_path);
}

void RobotController::init_logger()
{
  try
  {
    logger_ = std::make_unique<Logger>("robot_controller_logger", log_file_path);
    RCLCPP_INFO(this->get_logger(), "Logger initialized with file: %s", log_file_path.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger: %s", e.what());
    logger_.reset();
  }
}

void RobotController::update_logger()
{
  if (!logger_)
    return;

  try
  {
    // Skip if trajectory is empty
    if (trajectory_.poses.empty())
    {
      return;
    }

    // Calculate relative time from trajectory start
    rclcpp::Time current_time = this->now();
    double elapsed_seconds = current_time.seconds() - last_traj_update;
    rclcpp::Time relative_time = rclcpp::Time(trajectory_.poses[0].header.stamp) +
                                 rclcpp::Duration::from_seconds(elapsed_seconds);

    // Log RMSE between current pose and expected pose from trajectory
    logger_->log_current_rmse(trajectory_, current_pose_, relative_time);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error updating logger: %s", e.what());
  }
}

void RobotController::determine_current_waypoint()
{
  if (waypoints.empty() or current_waypoint_idx >= waypoints.size())
  {
    return;
  }

  double dpos_mag = std::hypot(
      waypoints[current_waypoint_idx].position(0) - robot_pose.position(0),
      waypoints[current_waypoint_idx].position(1) - robot_pose.position(1));

  while (dpos_mag < lookahead_dist and current_waypoint_idx < waypoints.size() - 1)
  {
    prev_arrival_time = this->now().nanoseconds() / 1e9 - last_traj_update;
    double real_time_difference = prev_arrival_time - hold_times.front();
    double traj_time_difference = hold_times[current_waypoint_idx] - hold_times.front();
    if (current_waypoint_idx > 3)
      real_time_factor = traj_time_difference / real_time_difference;
    else
      real_time_factor = 1.0;
    current_waypoint_idx++;
    dpos_mag = std::hypot(
        waypoints[current_waypoint_idx].position(0) - robot_pose.position(0),
        waypoints[current_waypoint_idx].position(1) - robot_pose.position(1));
  }
}

void RobotController::determine_mode()
{
  if (waypoints.empty() or current_waypoint_idx >= waypoints.size())
  {
    return;
  }

  double dx = waypoints[current_waypoint_idx].position(0) - robot_pose.position(0);
  double dy = waypoints[current_waypoint_idx].position(1) - robot_pose.position(1);

  Eigen::Vector3d dpos(dx, dy, 0);
  dpos = dpos.normalized();

  double curr_heading = Structs::Pose::get_yaw(robot_pose.orientation);
  double next_heading = Structs::Pose::get_yaw(waypoints[current_waypoint_idx].orientation);
  double dpos_angle = std::atan2(dy, dx);

  double now = this->now().seconds();
  double relative_now = now - last_traj_update;
  double hold_time = hold_times[current_waypoint_idx];

  bool backward = cos(normalize_angle_diff(dpos_angle, next_heading)) < 0.0; // 고쳐야 함.
  bool is_heading_aligned = cos(normalize_angle_diff(curr_heading, next_heading)) > 0.0;

  distance_difference = std::hypot(dx, dy);
  heading_difference = normalize_angle_diff(curr_heading, next_heading);

  if ((!backward and !is_heading_aligned) or !rotation_finished)
  {
    if (current_mode != ControlMode::ROTATE_IN_PLACE)
    {
      is_first_rotation_step = true;
      prev_heading_difference = 0.0;
    }
    current_mode = ControlMode::ROTATE_IN_PLACE;
    rotation_finished = false;
  }
  else if (waypoints.size() < 3 or waypoints.size() <= current_waypoint_idx or relative_now < hold_time)
  {
    current_mode = ControlMode::IDLE;
  }
  else if (backward)
  {
    current_mode = ControlMode::BACKWARD;
    path_difference = normalize_angle_diff(curr_heading + M_PI, dpos_angle);
  }
  else
  {
    current_mode = ControlMode::FORWARD;
    path_difference = normalize_angle_diff(curr_heading, dpos_angle);
  }
}

void RobotController::calculate_cmd_vel()
{
  if (waypoints.empty() or current_waypoint_idx >= waypoints.size())
  {
    return;
  }

  regulated_lin_vel = lin_vel;
  regulated_ang_vel = ang_vel;

  if (real_time_factor < 1.0 and real_time_factor > 0)
  {
    regulated_lin_vel = std::min(lin_vel / real_time_factor, max_lin_vel);
  }
  else if (real_time_factor > 1.0)
  {
    regulated_lin_vel = std::max(lin_vel / real_time_factor, min_lin_vel);
  }
  else
  {
    regulated_lin_vel = lin_vel;
  }

  double kappa = fabs(2 * sin(path_difference) / lookahead_dist);

  if (kappa * regulated_lin_vel > regulated_ang_vel && kappa > 0)
    regulated_lin_vel /= kappa;

  if (regulated_lin_vel < min_lin_vel && regulated_lin_vel != 0)
    regulated_lin_vel *= min_lin_vel / regulated_lin_vel;

  switch (current_mode)
  {
  case ControlMode::IDLE:
  {
    cmd_vel.linear = 0.0;
    cmd_vel.angular = 0.0;
    break;
  }

  case ControlMode::FORWARD:
  {
    cmd_vel.linear = regulated_lin_vel;
    cmd_vel.angular = kappa * regulated_lin_vel * (path_difference > 0 ? -1 : 1);
    break;
  }

  case ControlMode::BACKWARD:
  {
    cmd_vel.linear = -regulated_lin_vel;
    cmd_vel.angular = kappa * regulated_lin_vel * (path_difference > 0 ? -1 : 1);
    break;
  }

  case ControlMode::ROTATE_IN_PLACE:
  {
    if (is_first_rotation_step)
    {
      prev_heading_difference = heading_difference;
      is_first_rotation_step = false;
    }
    else if ((prev_heading_difference > 0 && heading_difference < 0) or (prev_heading_difference < 0 && heading_difference > 0))
    {
      rotation_finished = true;
    }
    prev_heading_difference = heading_difference;

    cmd_vel.linear = 0.0;
    cmd_vel.angular = regulated_ang_vel * (heading_difference > 0 ? -1 : 1);
    break;
  }
  }
}

RobotController::RobotController()
    : Node("robot_controller")
{
  set_params();
  init_logger();
  set_pubs_subs();
}

void RobotController::robot_controller_loop()
{
  determine_current_waypoint();
  determine_mode();
  calculate_cmd_vel();
  publish_cmd_vel();
  publish_traj_progress();
}
