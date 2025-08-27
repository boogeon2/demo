#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <yaml-cpp/yaml.h>

class Logger
{
public:
    Logger(const std::string &name, const std::string &log_file);
    ~Logger();

    void log_current_rmse(
        const nav_msgs::msg::Path &reference_trajectory,
        const geometry_msgs::msg::PoseStamped &current_pose,
        const rclcpp::Time &current_time);

    geometry_msgs::msg::PoseStamped get_near_trajectory(
        const nav_msgs::msg::Path &trajectory,
        const geometry_msgs::msg::PoseStamped &current_pose);

private:
    std::string name_;
    std::string log_file_;
    std::ofstream log_stream_;

    // RMSE values
    double position_rmse_;
    double orientation_rmse_;

    // Logger start time (for relative time calculation)
    double start_time_;

    // Helper functions
    double calculate_position_rmse(
        const geometry_msgs::msg::Point &expected,
        const geometry_msgs::msg::Point &actual);
    double calculate_orientation_rmse(
        const geometry_msgs::msg::Quaternion &expected,
        const geometry_msgs::msg::Quaternion &actual);
};
