#include "rmse_logger.hpp"

Logger::Logger(const std::string &name, const std::string &log_file)
    : name_(name), log_file_(log_file), position_rmse_(0.0), orientation_rmse_(0.0), start_time_(0.0)
{
  // Open log file
  log_stream_.open(log_file_, std::ios::out | std::ios::app);
  if (!log_stream_.is_open())
  {
    throw std::runtime_error("Failed to open log file: " + log_file_);
  }

  // Write header
  log_stream_ << "timestamp_seconds,position_rmse,orientation_rmse" << std::endl;
}

Logger::~Logger()
{
  // Close log file if open
  if (log_stream_.is_open())
  {
    log_stream_.close();
  }
}

void Logger::log_current_rmse(
    const nav_msgs::msg::Path &reference_trajectory,
    const geometry_msgs::msg::PoseStamped &current_pose,
    const rclcpp::Time &current_time)
{
  // Skip if reference trajectory is empty
  if (reference_trajectory.poses.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger(name_), "Reference trajectory is empty, cannot calculate RMSE");
    return;
  }

  // Initialize start time if it's the first log entry
  double current_time_sec = current_time.nanoseconds() / 1e9;
  if (start_time_ == 0.0)
  {
    start_time_ = current_time_sec;
    RCLCPP_INFO(rclcpp::get_logger(name_), "Starting log with reference time: %.2f seconds", start_time_);
  }

  // Calculate relative time from start
  double relative_time = current_time_sec - start_time_;

  // Get expected pose by interpolating the trajectory at current time
  geometry_msgs::msg::PoseStamped expected_pose =
      get_near_trajectory(reference_trajectory, current_pose);

  // Calculate position RMSE
  position_rmse_ = calculate_position_rmse(
      expected_pose.pose.position, current_pose.pose.position);

  // Calculate orientation RMSE
  orientation_rmse_ = calculate_orientation_rmse(
      expected_pose.pose.orientation, current_pose.pose.orientation);

  // Log the RMSE values with relative time (from start)
  log_stream_ << relative_time << ","
              << position_rmse_ << ","
              << orientation_rmse_ << std::endl;

  RCLCPP_DEBUG(
      rclcpp::get_logger(name_),
      "Time: %.2f, RMSE - Position: %.4f, Orientation: %.4f",
      relative_time, position_rmse_, orientation_rmse_);
}

geometry_msgs::msg::PoseStamped Logger::get_near_trajectory(
    const nav_msgs::msg::Path &trajectory,
    const geometry_msgs::msg::PoseStamped &current_pose)
{
  // return nearest pose from current pose
  if (trajectory.poses.empty())
  {
    throw std::runtime_error("Trajectory is empty, cannot find nearest pose");
  }
  double min_distance = std::numeric_limits<double>::max();
  size_t nearest_index = 0;
  for (size_t i = 0; i < trajectory.poses.size(); ++i)
  {
    double dx = trajectory.poses[i].pose.position.x - current_pose.pose.position.x;
    double dy = trajectory.poses[i].pose.position.y - current_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_index = i;
    }
  }
  // Return the nearest pose
  return trajectory.poses[nearest_index];
}

double Logger::calculate_position_rmse(
    const geometry_msgs::msg::Point &expected,
    const geometry_msgs::msg::Point &actual)
{
  double dx = expected.x - actual.x;
  double dy = expected.y - actual.y;
  double dz = expected.z - actual.z;

  // Calculate squared error
  double squared_error = dx * dx + dy * dy + dz * dz;

  // Return RMSE (square root of MSE)
  return std::sqrt(squared_error);
}

double Logger::calculate_orientation_rmse(
    const geometry_msgs::msg::Quaternion &expected,
    const geometry_msgs::msg::Quaternion &actual)
{
  // Convert quaternions to tf2 quaternions for easier manipulation
  tf2::Quaternion q_expected(
      expected.x,
      expected.y,
      expected.z,
      expected.w);

  tf2::Quaternion q_actual(
      actual.x,
      actual.y,
      actual.z,
      actual.w);

  // Normalize the quaternions to ensure they're valid
  q_expected.normalize();
  q_actual.normalize();

  // Calculate the dot product between the quaternions
  double dot_product = q_expected.dot(q_actual);

  // Clamp to valid range (due to potential floating point errors)
  dot_product = std::max(-1.0, std::min(1.0, dot_product));

  // The shortest angle between the orientations
  double angle_error = std::acos(2.0 * dot_product * dot_product - 1.0);

  // Return the angle error (RMSE is just the single error value in this case)
  return angle_error;
}
