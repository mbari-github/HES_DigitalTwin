#include "exoskeletron_safety_manager/SafeStopMode.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <chrono>

namespace functional_safety
{

void SafeStopMode::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  stop_client_ = node_->create_client<std_srvs::srv::SetBool>("/stop_robot");

  RCLCPP_INFO(node_->get_logger(), "SafeStopMode initialized");

  call_stop_service(true);
}

void SafeStopMode::stop()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "SafeStopMode stop");
  }
  call_stop_service(true);
}

void SafeStopMode::pause()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "SafeStopMode pause");
  }
}

void SafeStopMode::resume()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "SafeStopMode resume");
  }
  call_stop_service(true);
}

void SafeStopMode::shutdown()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "SafeStopMode shutdown");
  }

  call_stop_service(false);
}

void SafeStopMode::set_safety_params(double param)
{
  param_ = param;
  (void)param_;
}

void SafeStopMode::call_stop_service(bool enable)
{
  if (!node_ || !stop_client_) {
    return;
  }

  if (!stop_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SafeStopMode: /stop_robot service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  auto future = stop_client_->async_send_request(request);

  RCLCPP_INFO(
    node_->get_logger(),
    "SafeStopMode: sent /stop_robot request with data=%s",
    enable ? "true" : "false");

  (void)future;
}

}  // namespace functional_safety

PLUGINLIB_EXPORT_CLASS(functional_safety::SafeStopMode, functional_safety::SafetyTools)