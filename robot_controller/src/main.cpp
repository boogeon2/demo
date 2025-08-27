#include "robot_controller/robot_controller.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto robot_controller = std::make_shared<RobotController>();
  rclcpp::spin(robot_controller);
  rclcpp::shutdown();
  return 0;
}
