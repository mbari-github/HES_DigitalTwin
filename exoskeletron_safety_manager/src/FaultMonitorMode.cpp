#include "exoskeletron_safety_manager/FaultMonitorMode.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <chrono>

namespace functional_safety
{

void FaultMonitorMode::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  mode_request_sent_ = false;

  bridge_status_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/exo_bridge/status",
    10,
    std::bind(&FaultMonitorMode::bridge_status_callback, this, std::placeholders::_1));

  compliant_client_ = node_->create_client<std_srvs::srv::SetBool>("/compliant_mode_request");
  torque_limit_client_ = node_->create_client<std_srvs::srv::SetBool>("/torque_limit_request");
  safe_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("/safe_stop_request");

  RCLCPP_INFO(
    node_->get_logger(),
    "FaultMonitorMode initialized | tau thresholds = [%.3f, %.3f, %.3f] | vel thresholds = [%.3f, %.3f, %.3f]",
    tau_compliant_threshold_,
    tau_torque_limit_threshold_,
    tau_safe_stop_threshold_,
    vel_compliant_threshold_,
    vel_torque_limit_threshold_,
    vel_safe_stop_threshold_);
}

void FaultMonitorMode::stop()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode stop");
  }
}

void FaultMonitorMode::pause()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode pause");
  }
}

void FaultMonitorMode::resume()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode resume");
  }
}

void FaultMonitorMode::shutdown()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode shutdown");
  }

  bridge_status_sub_.reset();
  compliant_client_.reset();
  torque_limit_client_.reset();
  safe_stop_client_.reset();
}

void FaultMonitorMode::set_safety_params(double param)
{
  tau_safe_stop_threshold_ = param;

  if (node_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FaultMonitorMode safe stop tau threshold set to %.3f",
      tau_safe_stop_threshold_);
  }
}

FaultResponseLevel FaultMonitorMode::evaluate_tau(double tau_in) const
{
  const double abs_tau = std::abs(tau_in);

  if (abs_tau >= tau_safe_stop_threshold_) {
    return FaultResponseLevel::SAFE_STOP;
  }
  if (abs_tau >= tau_torque_limit_threshold_) {
    return FaultResponseLevel::TORQUE_LIMIT;
  }
  if (abs_tau >= tau_compliant_threshold_) {
    return FaultResponseLevel::COMPLIANT;
  }

  return FaultResponseLevel::NOMINAL;
}

FaultResponseLevel FaultMonitorMode::evaluate_velocity(double theta_dot) const
{
  const double abs_vel = std::abs(theta_dot);

  if (abs_vel >= vel_safe_stop_threshold_) {
    return FaultResponseLevel::SAFE_STOP;
  }
  if (abs_vel >= vel_torque_limit_threshold_) {
    return FaultResponseLevel::TORQUE_LIMIT;
  }
  if (abs_vel >= vel_compliant_threshold_) {
    return FaultResponseLevel::COMPLIANT;
  }

  return FaultResponseLevel::NOMINAL;
}

FaultResponseLevel FaultMonitorMode::max_level(
  FaultResponseLevel a,
  FaultResponseLevel b) const
{
  return (static_cast<int>(a) >= static_cast<int>(b)) ? a : b;
}

void FaultMonitorMode::bridge_status_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!node_) {
    return;
  }

  // Layout /exo_bridge/status:
  // [0] stop_active
  // [1] override_active
  // [2] theta
  // [3] theta_dot
  // [4] theta_hold
  // [5] tau_in
  // [6] tau_ext
  // [7] control_mode_id
  if (msg->data.size() < 8) {
    RCLCPP_WARN(node_->get_logger(), "FaultMonitorMode: /exo_bridge/status malformed");
    return;
  }

  if (mode_request_sent_) {
    return;
  }

  const double theta_dot = msg->data[3];
  const double tau_in = msg->data[5];

  const auto tau_level = evaluate_tau(tau_in);
  const auto vel_level = evaluate_velocity(theta_dot);
  const auto final_level = max_level(tau_level, vel_level);

  switch (final_level) {
    case FaultResponseLevel::SAFE_STOP:
      RCLCPP_WARN(
        node_->get_logger(),
        "FaultMonitorMode: SAFE_STOP requested | tau_in=%.3f theta_dot=%.3f",
        tau_in, theta_dot);
      request_safe_stop();
      mode_request_sent_ = true;
      break;

    case FaultResponseLevel::TORQUE_LIMIT:
      RCLCPP_WARN(
        node_->get_logger(),
        "FaultMonitorMode: TORQUE_LIMIT requested | tau_in=%.3f theta_dot=%.3f",
        tau_in, theta_dot);
      request_torque_limit_mode();
      mode_request_sent_ = true;
      break;

    case FaultResponseLevel::COMPLIANT:
      RCLCPP_WARN(
        node_->get_logger(),
        "FaultMonitorMode: COMPLIANT requested | tau_in=%.3f theta_dot=%.3f",
        tau_in, theta_dot);
      request_compliant_mode();
      mode_request_sent_ = true;
      break;

    case FaultResponseLevel::NOMINAL:
    default:
      break;
  }
}

void FaultMonitorMode::request_compliant_mode()
{
  if (!node_ || !compliant_client_) {
    return;
  }

  if (!compliant_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(node_->get_logger(), "FaultMonitorMode: /compliant_mode_request service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  auto future = compliant_client_->async_send_request(request);
  (void)future;

  RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode: sent /compliant_mode_request");
}

void FaultMonitorMode::request_torque_limit_mode()
{
  if (!node_ || !torque_limit_client_) {
    return;
  }

  if (!torque_limit_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(node_->get_logger(), "FaultMonitorMode: /torque_limit_request service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  auto future = torque_limit_client_->async_send_request(request);
  (void)future;

  RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode: sent /torque_limit_request");
}

void FaultMonitorMode::request_safe_stop()
{
  if (!node_ || !safe_stop_client_) {
    return;
  }

  if (!safe_stop_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(node_->get_logger(), "FaultMonitorMode: /safe_stop_request service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  auto future = safe_stop_client_->async_send_request(request);
  (void)future;

  RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode: sent /safe_stop_request");
}

}  // namespace functional_safety

PLUGINLIB_EXPORT_CLASS(functional_safety::FaultMonitorMode, functional_safety::SafetyTools)