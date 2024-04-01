#include "implementation/cup_simulator.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CupSimulator>());
  rclcpp::shutdown();
  return 0;
}