#include "global_planner/global_planner.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<Server>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}
