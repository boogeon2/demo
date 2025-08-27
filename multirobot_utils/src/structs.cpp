#include "multirobot_utils/structs.hpp"

Structs::Pose::Pose()
{
  position = Eigen::Vector3d(0.0, 0.0, 0.0);
  orientation = Eigen::Vector3d(0.0, 0.0, 0.0);
}

Structs::Pose::Pose(Eigen::Vector3d position_, Eigen::Vector3d orientation_)
{
  position = position_;
  orientation = orientation_;
}

Structs::Pose::Pose(double x_, double y_, double theta_)
{
  position = Eigen::Vector3d(x_, y_, 0.0);
  orientation = Eigen::Vector3d(cos(theta_), sin(theta_), 0.0);
}

Structs::Pose::Pose(const geometry_msgs::msg::Pose &pose_msg)
{
  position = Eigen::Vector3d(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
  orientation = Eigen::Vector3d(cos(get_yaw(pose_msg.orientation)), sin(get_yaw(pose_msg.orientation)), 0.0);
}

Structs::Pose::Pose(const geometry_msgs::msg::PoseStamped &pose_msg)
{
  position = Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
  orientation = Eigen::Vector3d(cos(get_yaw(pose_msg.pose.orientation)), sin(get_yaw(pose_msg.pose.orientation)), 0.0);
}

double Structs::Pose::get_yaw()
{
  return atan2(orientation[1], orientation[0]);
}

double Structs::Pose::get_yaw(geometry_msgs::msg::Quaternion q)
{
  tf2::Quaternion tf2_q;
  tf2::fromMsg(q, tf2_q);

  double roll, pitch, yaw;
  tf2::Matrix3x3 m(tf2_q);
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

double Structs::Pose::get_yaw(Eigen::Vector3d orientation)
{
  return atan2(orientation[1], orientation[0]);
}

tf2::Quaternion Structs::Pose::get_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

void Structs::Pose::rotate(double yaw)
{
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  orientation = rotation_matrix * orientation;
}

double Structs::Pose::norm()
{
  return position.norm();
}

geometry_msgs::msg::Pose Structs::Pose::to_msg()
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = position[0];
  pose_msg.position.y = position[1];
  pose_msg.position.z = position[2];
  double yaw = get_yaw(orientation);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose_msg.orientation = tf2::toMsg(q);

  return pose_msg;
}

Structs::Trajectory::Waypoint::Waypoint()
{
  time = 0.0;
  pose = Pose();
  velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
}

Structs::Trajectory::Waypoint::Waypoint(double time_, Pose pose_, Eigen::Vector3d velocity_)
{
  time = time_;
  pose = pose_;
  velocity = velocity_;
}

Structs::Trajectory::Trajectory()
{
  trajectory.clear();
  trajectory = {};
}

Structs::Trajectory::Trajectory(const Trajectory &trajectory_)
{
  trajectory.clear();
  trajectory = trajectory_.trajectory;
}

Structs::Trajectory::Trajectory(std::map<double, Waypoint> trajectory_)
{
  trajectory.clear();
  trajectory = trajectory_;
}

Structs::Trajectory::Trajectory(Waypoint waypoint_)
{
  trajectory.clear();
  trajectory[waypoint_.time] = waypoint_;
}

int Structs::Trajectory::size() const
{
  return trajectory.size();
}

Structs::Trajectory::wp_iterator Structs::Trajectory::begin()
{
  return trajectory.begin();
}

Structs::Trajectory::wp_iterator Structs::Trajectory::end()
{
  return trajectory.end();
}

Structs::Trajectory::wp_iterator Structs::Trajectory::find(double time)
{
  return trajectory.find(time);
}

void Structs::Trajectory::insert(double time_, Pose pose_, Eigen::Vector3d velocity_)
{
  Waypoint new_waypoint(time_, pose_, velocity_);
  trajectory[time_] = new_waypoint;
}

double Structs::Trajectory::start_time()
{
  return trajectory.begin()->first;
}

double Structs::Trajectory::finish_time()
{
  return trajectory.rbegin()->first;
}

void Structs::Trajectory::adjust_times(double delta_t)
{
  std::map<double, Waypoint> new_trajectory;
  for (auto &waypoint : trajectory)
  {
    waypoint.second.time += delta_t;
    new_trajectory[waypoint.second.time] = waypoint.second;
  }
  trajectory.clear();
  trajectory = new_trajectory;
}
