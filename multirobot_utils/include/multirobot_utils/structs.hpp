#pragma once

// C++ Standard Libraries
#include "cmath"
#include "limits"
#include "map"
#include "vector"
#include "memory"
#include "optional"
#include "cassert"

// ROS2 Libraries
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// External Libraries
#ifdef MULTIROBOT_UTILS__USING_FCL_0_6
#include "fcl/math/motion/spline_motion.h"
#include "fcl/narrowphase/collision_object.h"
#else
#include "fcl/ccd/motion.h"
#include "fcl/collision_object.h"
#endif
#include "Eigen/Dense"

class Structs
{
public:
  double PI = 3.14159265358979323846;
  double INF = std::numeric_limits<double>::infinity();

  struct Pose
  {
    // Variables
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;

    // Constructors
    Pose();
    Pose(double x_, double y_, double theta_);
    Pose(Eigen::Vector3d position_, Eigen::Vector3d orientation_);
    Pose(const geometry_msgs::msg::Pose &pose_msg);
    Pose(const geometry_msgs::msg::PoseStamped &pose_msg);

    // Functions
    double get_yaw();
    static double get_yaw(geometry_msgs::msg::Quaternion q);
    static double get_yaw(Eigen::Vector3d orientation);
    static tf2::Quaternion get_quaternion(double yaw);
    void rotate(double yaw);
    double norm();
    geometry_msgs::msg::Pose to_msg();
  };

  struct Velocity
  {
    double linear;
    double angular;

    Velocity() : linear(0.0), angular(0.0) {}
    Velocity(double lin, double ang) : linear(lin), angular(ang) {}
  };

  struct Trajectory
  {
    struct Waypoint
    {
      // Variables
      double time;
      Pose pose;
      Eigen::Vector3d velocity;

      // Constructors
      Waypoint();
      Waypoint(double time_, Pose pose_, Eigen::Vector3d velocity_);
    };

    // Typedefs
    using wp_iterator = std::map<double, Waypoint>::iterator;

    // Variables
    std::map<double, Waypoint> trajectory;

    // Constructors
    Trajectory();
    Trajectory(const Trajectory &trajectory_);
    Trajectory(std::map<double, Waypoint> trajectory_);
    Trajectory(Waypoint waypoint_);

    // Functions
    int size() const;
    wp_iterator begin();
    wp_iterator end();
    wp_iterator find(double time);
    void insert(double time_, Pose pose_, Eigen::Vector3d velocity_);
    double start_time();
    double finish_time();
    void adjust_times(double delta_t);
  };
};
