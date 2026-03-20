#include "exoskeletron_safety_manager/SensorDegradedMode.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace functional_safety
{

void SensorDegradedMode::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  RCLCPP_INFO(node_->get_logger(), "SensorDegradedMode initialized");
}

void SensorDegradedMode::stop() {}
void SensorDegradedMode::pause() {}
void SensorDegradedMode::resume() {}
void SensorDegradedMode::shutdown() {}

void SensorDegradedMode::set_safety_params(double param) {}

}

PLUGINLIB_EXPORT_CLASS(functional_safety::SensorDegradedMode, functional_safety::SafetyTools)