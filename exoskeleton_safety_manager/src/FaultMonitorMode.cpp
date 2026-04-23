#include "exoskeleton_safety_manager/FaultMonitorMode.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <chrono>

namespace functional_safety
{

void FaultMonitorMode::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;

  last_requested_level_ = FaultResponseLevel::NOMINAL;
  compliant_counter_ = 0;
  torque_counter_ = 0;
  stop_counter_ = 0;
  message_received_ = false;
  watchdog_triggered_ = false;

  // ---- Declare parameters on the host node (declare_if_missing avoids
  //      duplicate-declaration errors when the plugin is reloaded) ----
  auto declare_if_missing = [&](const std::string & name, double default_val) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_val);
    }
  };

  // Escalation entry thresholds
  declare_if_missing("fault_monitor.tau_compliant_threshold",          2.0);
  declare_if_missing("fault_monitor.tau_torque_limit_threshold",       3.0);
  declare_if_missing("fault_monitor.tau_safe_stop_threshold",          4.0);
  declare_if_missing("fault_monitor.vel_compliant_threshold",          1.0);
  declare_if_missing("fault_monitor.vel_torque_limit_threshold",       2.0);
  declare_if_missing("fault_monitor.vel_safe_stop_threshold",          3.0);

  // Watchdog timeout
  declare_if_missing("fault_monitor.status_timeout_seconds",           0.5);

  // Debounce thresholds (number of consecutive samples above threshold)
  declare_if_missing("fault_monitor.compliant_count_threshold",        3.0);
  declare_if_missing("fault_monitor.torque_count_threshold",           3.0);
  declare_if_missing("fault_monitor.stop_count_threshold",             2.0);

  // Grace period after initialization before monitoring starts
  declare_if_missing("fault_monitor.grace_period_seconds",             2.0);

  load_thresholds_from_params();

  // ---- Start grace period ----
  grace_period_active_ = true;
  grace_period_end_ = node_->now() +
    rclcpp::Duration::from_seconds(grace_period_seconds_);

  // Set the bridge to 'nominal' when entering FAULT_MONITOR.
  // This is the single point of responsibility for setting the bridge mode:
  // - At startup: this call initializes the bridge to nominal.
  // - After reset from SAFE_STOP: this call restores nominal.
  // If this initialization were left to the outgoing plugin's shutdown(), it would
  // create a dangerous race condition: the bridge could briefly be in nominal mode
  // before the new plugin's initialize() runs.
  init_bridge_client(node);
  request_mode("nominal");

  RCLCPP_INFO(
    node_->get_logger(),
    "FaultMonitorMode initialized | "
    "tau=[%.2f, %.2f, %.2f] vel=[%.2f, %.2f, %.2f] "
    "timeout=%.2fs | grace_period=%.1fs",
    tau_compliant_threshold_,
    tau_torque_limit_threshold_,
    tau_safe_stop_threshold_,
    vel_compliant_threshold_,
    vel_torque_limit_threshold_,
    vel_safe_stop_threshold_,
    status_timeout_seconds_,
    grace_period_seconds_);

  // ---- Subscriber and service clients ----
  bridge_status_sub_ = node_->create_subscription<exoskeleton_safety_msgs::msg::Float64ArrayStamped>(
    "/exo_bridge/status",
    10,
    std::bind(&FaultMonitorMode::bridge_status_callback, this, std::placeholders::_1));

  compliant_client_    = node_->create_client<std_srvs::srv::SetBool>("/compliant_mode_request");
  torque_limit_client_ = node_->create_client<std_srvs::srv::SetBool>("/torque_limit_request");
  safe_stop_client_    = node_->create_client<std_srvs::srv::SetBool>("/safe_stop_request");

  watchdog_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&FaultMonitorMode::watchdog_callback, this));
}

void FaultMonitorMode::load_thresholds_from_params()
{
  if (!node_) return;

  tau_compliant_threshold_    = node_->get_parameter("fault_monitor.tau_compliant_threshold").as_double();
  tau_torque_limit_threshold_ = node_->get_parameter("fault_monitor.tau_torque_limit_threshold").as_double();
  tau_safe_stop_threshold_    = node_->get_parameter("fault_monitor.tau_safe_stop_threshold").as_double();
  vel_compliant_threshold_    = node_->get_parameter("fault_monitor.vel_compliant_threshold").as_double();
  vel_torque_limit_threshold_ = node_->get_parameter("fault_monitor.vel_torque_limit_threshold").as_double();
  vel_safe_stop_threshold_    = node_->get_parameter("fault_monitor.vel_safe_stop_threshold").as_double();
  status_timeout_seconds_     = node_->get_parameter("fault_monitor.status_timeout_seconds").as_double();
  compliant_count_threshold_  = static_cast<int>(
    node_->get_parameter("fault_monitor.compliant_count_threshold").as_double());
  torque_count_threshold_     = static_cast<int>(
    node_->get_parameter("fault_monitor.torque_count_threshold").as_double());
  stop_count_threshold_       = static_cast<int>(
    node_->get_parameter("fault_monitor.stop_count_threshold").as_double());
  grace_period_seconds_       = node_->get_parameter("fault_monitor.grace_period_seconds").as_double();
}

void FaultMonitorMode::stop()
{
  if (node_) RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode stop");
}

void FaultMonitorMode::pause()
{
  if (node_) RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode pause");
}

void FaultMonitorMode::resume()
{
  if (node_) RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode resume");
}

void FaultMonitorMode::shutdown()
{
  if (node_) RCLCPP_INFO(node_->get_logger(), "FaultMonitorMode shutdown");
  // Release all ROS resources explicitly to avoid dangling callbacks
  bridge_status_sub_.reset();
  watchdog_timer_.reset();
  compliant_client_.reset();
  torque_limit_client_.reset();
  safe_stop_client_.reset();
}

void FaultMonitorMode::set_safety_params(double param)
{
  tau_safe_stop_threshold_ = param;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
      "FaultMonitorMode: tau_safe_stop_threshold set to %.3f",
      tau_safe_stop_threshold_);
  }
}

// ============================================================
// Escalation evaluation
// ============================================================

FaultResponseLevel FaultMonitorMode::evaluate_tau_entry(double tau_in) const
{
  const double abs_tau = std::abs(tau_in);
  if (abs_tau >= tau_safe_stop_threshold_)    return FaultResponseLevel::SAFE_STOP;
  if (abs_tau >= tau_torque_limit_threshold_) return FaultResponseLevel::TORQUE_LIMIT;
  if (abs_tau >= tau_compliant_threshold_)    return FaultResponseLevel::COMPLIANT;
  return FaultResponseLevel::NOMINAL;
}

FaultResponseLevel FaultMonitorMode::evaluate_velocity_entry(double theta_dot) const
{
  const double abs_vel = std::abs(theta_dot);
  if (abs_vel >= vel_safe_stop_threshold_)    return FaultResponseLevel::SAFE_STOP;
  if (abs_vel >= vel_torque_limit_threshold_) return FaultResponseLevel::TORQUE_LIMIT;
  if (abs_vel >= vel_compliant_threshold_)    return FaultResponseLevel::COMPLIANT;
  return FaultResponseLevel::NOMINAL;
}

FaultResponseLevel FaultMonitorMode::max_level(
  FaultResponseLevel a, FaultResponseLevel b) const
{
  return (static_cast<int>(a) >= static_cast<int>(b)) ? a : b;
}

bool FaultMonitorMode::is_more_severe(
  FaultResponseLevel candidate, FaultResponseLevel current) const
{
  return static_cast<int>(candidate) > static_cast<int>(current);
}

// ============================================================
// Debounce
// ============================================================

void FaultMonitorMode::update_debounce_counters(FaultResponseLevel level)
{
  if (level >= FaultResponseLevel::COMPLIANT)    ++compliant_counter_;  else compliant_counter_ = 0;
  if (level >= FaultResponseLevel::TORQUE_LIMIT) ++torque_counter_;     else torque_counter_ = 0;
  if (level >= FaultResponseLevel::SAFE_STOP)    ++stop_counter_;       else stop_counter_ = 0;
}

FaultResponseLevel FaultMonitorMode::debounced_entry_level() const
{
  if (stop_counter_      >= stop_count_threshold_)      return FaultResponseLevel::SAFE_STOP;
  if (torque_counter_    >= torque_count_threshold_)     return FaultResponseLevel::TORQUE_LIMIT;
  if (compliant_counter_ >= compliant_count_threshold_)  return FaultResponseLevel::COMPLIANT;
  return FaultResponseLevel::NOMINAL;
}

// ============================================================
// Bridge status callback
// ============================================================

void FaultMonitorMode::bridge_status_callback(
  const exoskeleton_safety_msgs::msg::Float64ArrayStamped::SharedPtr msg)
{
  if (!node_ || msg->data.size() < 9) return;

  // Suppress escalation during the startup grace period
  if (grace_period_active_) {
    if (node_->now() < grace_period_end_) {
      return;
    }
    grace_period_active_ = false;
    compliant_counter_ = 0;
    torque_counter_ = 0;
    stop_counter_ = 0;
    last_requested_level_ = FaultResponseLevel::NOMINAL;
    RCLCPP_INFO(node_->get_logger(),
      "FaultMonitorMode: grace period ended, monitoring active");
  }

  message_received_ = true;
  watchdog_triggered_ = false;
  last_msg_time_ = node_->now();

  const double theta_dot = msg->data[3];
  // Escalation uses tau_out (data[5]): torque actually applied to the plant.
  // This is correct because FaultMonitorMode evaluates whether the plant is
  // experiencing a dangerous torque, not what the controller wants to apply.
  // tau_raw (data[8]) is used only by StateMachine for downgrade decisions.
  const double tau_in    = msg->data[5];

  const auto tau_level     = evaluate_tau_entry(tau_in);
  const auto vel_level     = evaluate_velocity_entry(theta_dot);
  const auto instantaneous = max_level(tau_level, vel_level);

  update_debounce_counters(instantaneous);
  const auto entry_level = debounced_entry_level();

  // Escalation is monotone: do not re-request a level already requested
  if (!is_more_severe(entry_level, last_requested_level_)) {
    return;
  }

  switch (entry_level) {
    case FaultResponseLevel::SAFE_STOP:
      RCLCPP_ERROR(node_->get_logger(),
        "FaultMonitorMode: SAFE_STOP | tau=%.3f vel=%.3f",
        tau_in, theta_dot);
      request_safe_stop();
      last_requested_level_ = FaultResponseLevel::SAFE_STOP;
      break;

    case FaultResponseLevel::TORQUE_LIMIT:
      RCLCPP_WARN(node_->get_logger(),
        "FaultMonitorMode: TORQUE_LIMIT | tau=%.3f vel=%.3f",
        tau_in, theta_dot);
      request_torque_limit_mode();
      last_requested_level_ = FaultResponseLevel::TORQUE_LIMIT;
      break;

    case FaultResponseLevel::COMPLIANT:
      RCLCPP_WARN(node_->get_logger(),
        "FaultMonitorMode: COMPLIANT | tau=%.3f vel=%.3f",
        tau_in, theta_dot);
      request_compliant_mode();
      last_requested_level_ = FaultResponseLevel::COMPLIANT;
      break;

    default:
      break;
  }
}

// ============================================================
// Watchdog
// ============================================================

void FaultMonitorMode::watchdog_callback()
{
  if (!node_ || !message_received_) return;

  if (grace_period_active_) return;

  const double elapsed = (node_->now() - last_msg_time_).seconds();
  if (elapsed < status_timeout_seconds_) return;
  if (watchdog_triggered_) return;

  watchdog_triggered_ = true;
  RCLCPP_ERROR(node_->get_logger(),
    "FaultMonitorMode watchdog: /exo_bridge/status timeout (%.3fs)", elapsed);

  if (is_more_severe(FaultResponseLevel::SAFE_STOP, last_requested_level_)) {
    request_safe_stop();
    last_requested_level_ = FaultResponseLevel::SAFE_STOP;
  }
}

// ============================================================
// Service calls
// ============================================================

bool FaultMonitorMode::call_bool_service(
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client,
  const std::string & service_name,
  bool value)
{
  if (!node_ || !client) {
    RCLCPP_ERROR(rclcpp::get_logger("FaultMonitorMode"),
      "call_bool_service: null client for %s", service_name.c_str());
    return false;
  }

  if (!client->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_ERROR(node_->get_logger(),
      "FaultMonitorMode: service %s not available", service_name.c_str());
    return false;
  }

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = value;

  auto future_cb = [this, service_name](
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f)
  {
    try {
      auto res = f.get();
      if (!res->success) {
        RCLCPP_WARN(node_->get_logger(),
          "FaultMonitorMode: %s success=false: %s",
          service_name.c_str(), res->message.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(),
        "FaultMonitorMode: exception in response to %s: %s",
        service_name.c_str(), e.what());
    }
  };

  client->async_send_request(req, future_cb);
  return true;
}

void FaultMonitorMode::request_compliant_mode()
{
  call_bool_service(compliant_client_, "/compliant_mode_request", true);
}

void FaultMonitorMode::request_torque_limit_mode()
{
  call_bool_service(torque_limit_client_, "/torque_limit_request", true);
}

void FaultMonitorMode::request_safe_stop()
{
  call_bool_service(safe_stop_client_, "/safe_stop_request", true);
}

}  // namespace functional_safety

PLUGINLIB_EXPORT_CLASS(
  functional_safety::FaultMonitorMode,
  functional_safety::SafetyTools)
