#include "local_planner/local_planner.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto robot = std::make_shared<Robot>();
  rclcpp::spin(robot);
  rclcpp::shutdown();
  return 0;
}
