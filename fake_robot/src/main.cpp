#include "fake_robot/fake_robot.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto fake_robot = std::make_shared<FakeRobot>();
  rclcpp::spin(fake_robot);
  rclcpp::shutdown();
  return 0;
}
