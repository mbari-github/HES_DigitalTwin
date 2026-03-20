#include "exoskeletron_safety_manager/TorqueLimitMode.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <chrono>

namespace functional_safety
{

void TorqueLimitMode::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  override_client_ = node_->create_client<std_srvs::srv::SetBool>("/override_control");

  RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode initialized");

  call_override_service(true);
}

void TorqueLimitMode::stop()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode stop");
  }
  call_override_service(true);
}

void TorqueLimitMode::pause()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode pause");
  }
}

void TorqueLimitMode::resume()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode resume");
  }
  call_override_service(true);
}

void TorqueLimitMode::shutdown()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode shutdown");
  }

  call_override_service(false);
}

void TorqueLimitMode::set_safety_params(double param)
{
  param_ = param;
  (void)param_;
}

void TorqueLimitMode::call_override_service(bool enable)
{
  if (!node_ || !override_client_) {
    return;
  }

  if (!override_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(
      node_->get_logger(),
      "TorqueLimitMode: /override_control service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  auto future = override_client_->async_send_request(request);

  RCLCPP_INFO(
    node_->get_logger(),
    "TorqueLimitMode: sent /override_control request with data=%s",
    enable ? "true" : "false");

  (void)future;
}

}  // namespace functional_safety

PLUGINLIB_EXPORT_CLASS(functional_safety::TorqueLimitMode, functional_safety::SafetyTools)