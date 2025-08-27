#include "fake_robot/fake_robot.hpp"

void FakeRobot::set_pubs_subs()
{
  // Subscribers
  initial_pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&FakeRobot::register_robot, this, std::placeholders::_1));

  cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&FakeRobot::get_cmd_vel, this, std::placeholders::_1));

  // Publishers
  pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  // TF Broadcaster
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Timer
  fake_robot_timer = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
      std::bind(&FakeRobot::fake_robot_loop, this));
}

void FakeRobot::register_robot(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "로봇 등록 [%s]...", robot_name.c_str());

  robot_pose.position = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

  tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  q.normalize();

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose.orientation = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0);

  initialized = true;
  RCLCPP_INFO(get_logger(), "로봇 [%s] 등록 완료, 위치: [%.2f, %.2f]",
              robot_name.c_str(), robot_pose.position.x(), robot_pose.position.y());
}

void FakeRobot::get_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_cmd_vel = *msg;
  last_cmd_vel_time = now();
}

void FakeRobot::publish_pose()
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = robot_pose.position.x();
  pose_msg.pose.position.y = robot_pose.position.y();
  pose_msg.pose.position.z = robot_pose.position.z();

  double yaw = std::atan2(robot_pose.orientation.y(), robot_pose.orientation.x());
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_pub->publish(pose_msg);
}

void FakeRobot::publish_tf()
{
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = now();
  t.header.frame_id = "map";
  t.child_frame_id = robot_name + "/base_link";

  t.transform.translation.x = robot_pose.position.x();
  t.transform.translation.y = robot_pose.position.y();
  t.transform.translation.z = robot_pose.position.z();

  double yaw = std::atan2(robot_pose.orientation.y(), robot_pose.orientation.x());
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster->sendTransform(t);
}

void FakeRobot::set_params()
{
  declare_parameter("robot_name", robot_name);
  declare_parameter("update_rate", update_rate);
  declare_parameter("timeout", timeout);

  get_parameter("robot_name", robot_name);
  get_parameter("update_rate", update_rate);
  get_parameter("timeout", timeout);
}

void FakeRobot::update_pose()
{
  // cmd_vel 타임아웃 확인
  auto current_time = now();
  auto time_diff = (current_time - last_cmd_vel_time).seconds();

  if (time_diff > timeout)
  {
    // cmd_vel이 타임아웃되면 로봇 정지
    current_cmd_vel.linear.x = 0.0;
    current_cmd_vel.linear.y = 0.0;
    current_cmd_vel.angular.z = 0.0;
  }

  // 타임스텝 계산
  double dt = 1.0 / update_rate;

  // 현재 방향 (yaw) 계산
  double yaw = std::atan2(robot_pose.orientation.y(), robot_pose.orientation.x());

  // 방향 업데이트
  yaw += current_cmd_vel.angular.z * dt;
  robot_pose.orientation = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);

  // 위치 업데이트 - 로봇의 로컬 프레임 기준
  double dx = current_cmd_vel.linear.x * std::cos(yaw);
  double dy = current_cmd_vel.linear.x * std::sin(yaw);

  robot_pose.position.x() += dx * dt;
  robot_pose.position.y() += dy * dt;
}

FakeRobot::FakeRobot()
    : Node("fake_robot")
{
  set_params();
  set_pubs_subs();

  // 초기 타임스탬프 설정
  last_cmd_vel_time = now();

  RCLCPP_INFO(get_logger(), "FakeRobot 노드 시작 - 로봇 이름: %s", robot_name.c_str());
  RCLCPP_INFO(get_logger(), "info 메시지 대기 중... 초기화 후 pose 발행을 시작합니다.");
}

void FakeRobot::fake_robot_loop()
{
  // 로봇이 초기화된 경우에만 처리
  if (initialized)
  {
    update_pose();
    publish_pose();
    publish_tf();
  }
}
