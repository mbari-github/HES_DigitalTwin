#include "exoskeletron_safety_manager/StateMachine.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<functional_safety::StateMachine>();
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}