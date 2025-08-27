#include "local_planner/local_planner.hpp"

void Robot::set_pubs_subs()
{
  traj_sub = create_subscription<nav_msgs::msg::Path>(
      "server_traj", 10, std::bind(&Robot::get_traj, this, std::placeholders::_1));

  pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", 10, std::bind(&Robot::update_pose, this, std::placeholders::_1));

  traj_pub = create_publisher<nav_msgs::msg::Path>(
      "traj", 10);

  delayed_time_pub = create_publisher<std_msgs::msg::Float64>(
      "delayed_time", 10);

  robot_timer = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&Robot::robot_loop, this));
}

void Robot::get_traj(const nav_msgs::msg::Path::SharedPtr msg)
{
  traj_msg = msg;

  if (!initialized)
    return;

  RCLCPP_INFO(get_logger(), "서버로부터 새 경로 수신 [%s]", robot_name.c_str());

  // 기존 경로 정보 초기화
  received_traj.clear();

  // 시작 시간 계산
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

    Structs::Pose pose;
    pose.position = Eigen::Vector3d(position.x(), position.y(), 0.0);
    pose.orientation = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);

    received_traj[relative_time] = pose;
  }

  last_traj_received = now().seconds();

  // 로컬 경로 업데이트
  update_local_plan();
}

void Robot::update_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

  if (!initialized and robot_pose.position.norm() > 0.1)
  {
    initialized = true;
    RCLCPP_INFO(get_logger(), "로봇 [%s] 초기화 완료", robot_name.c_str());
  }
}

void Robot::publish_delayed_time()
{
  if (!initialized)
    return;

  std_msgs::msg::Float64 delay_msg;
  delay_msg.data = delayed_time;
  delayed_time_pub->publish(delay_msg);
}

void Robot::set_params()
{
  declare_parameter("robot_name", robot_name);
  declare_parameter("v_max", v_max);
  declare_parameter("w_max", w_max);

  get_parameter("robot_name", robot_name);
  get_parameter("v_max", v_max);
  get_parameter("w_max", w_max);
}

void Robot::update_local_plan()
{
  // 현재는 개발 전 단계이므로 수신한 경로를 그대로 활용
  updated_traj = received_traj;

  // 경로 발행
  if (!updated_traj.empty())
  {
    traj_pub->publish(*traj_msg);
    last_traj_update = now().seconds();
  }
}

void Robot::calculate_delayed_time()
{
  // 경로가 비어있는 경우 처리
  if (received_traj.size() < 2)
  {
    delayed_time = 0.0;
    return;
  }

  // delayed_time 계산
  delayed_time = 0.0;
}

Robot::Robot()
    : Node("local_planner")
{
  set_params();
  set_pubs_subs();

  RCLCPP_INFO(get_logger(), "MRS Robot 노드 시작 - 로봇 이름: %s", robot_name.c_str());
  RCLCPP_INFO(get_logger(), "info 메시지 대기 중... 초기화 후 traj 발행을 시작합니다.");
}

void Robot::robot_loop()
{
  calculate_delayed_time();
  publish_delayed_time();
}
