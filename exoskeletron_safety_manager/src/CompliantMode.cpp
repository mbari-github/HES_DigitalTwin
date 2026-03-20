#include "exoskeletron_safety_manager/CompliantMode.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <chrono>

namespace functional_safety
{

void CompliantMode::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  compliant_client_ = node_->create_client<std_srvs::srv::SetBool>("/compliant_control");

  RCLCPP_INFO(node_->get_logger(), "CompliantMode initialized");

  call_compliant_service(true);
}

void CompliantMode::stop()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "CompliantMode stop");
  }
  call_compliant_service(true);
}

void CompliantMode::pause()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "CompliantMode pause");
  }
}

void CompliantMode::resume()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "CompliantMode resume");
  }
  call_compliant_service(true);
}

void CompliantMode::shutdown()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "CompliantMode shutdown");
  }

  call_compliant_service(false);
}

void CompliantMode::set_safety_params(double param)
{
  param_ = param;
  (void)param_;
}

void CompliantMode::call_compliant_service(bool enable)
{
  if (!node_ || !compliant_client_) {
    return;
  }

  if (!compliant_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CompliantMode: /compliant_control service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  auto future = compliant_client_->async_send_request(request);

  RCLCPP_INFO(
    node_->get_logger(),
    "CompliantMode: sent /compliant_control request with data=%s",
    enable ? "true" : "false");

  (void)future;
}

}  // namespace functional_safety

PLUGINLIB_EXPORT_CLASS(functional_safety::CompliantMode, functional_safety::SafetyTools)