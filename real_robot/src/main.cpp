#include "real_robot/real_robot.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto isr_m2 = std::make_shared<ISR_M2>();
    rclcpp::spin(isr_m2);
    rclcpp::shutdown();
    return 0;
}
