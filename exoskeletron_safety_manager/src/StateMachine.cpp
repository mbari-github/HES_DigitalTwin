#include "exoskeletron_safety_manager/StateMachine.hpp"

namespace functional_safety
{

StateMachine::StateMachine()
: Node("state_machine"),
  state_loader_("exoskeletron_safety_manager", "functional_safety::SafetyTools")
{
}

void StateMachine::initialize()
{
  current_state_ = functional_safety::SafetyState::FAULT_MONITOR;

  if (!change_state("FaultMonitorMode")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize in FAULT_MONITOR");
  }

  safe_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
    "safe_stop_request",
    std::bind(
      &StateMachine::safe_stop_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  compliant_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
    "compliant_mode_request",
    std::bind(
      &StateMachine::compliant_mode_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  torque_limit_service_ = this->create_service<std_srvs::srv::SetBool>(
    "torque_limit_request",
    std::bind(
      &StateMachine::torque_limit_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  reset_safety_service_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_safety_request",
    std::bind(
      &StateMachine::reset_safety_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "StateMachine services ready");
}

bool StateMachine::change_state(const std::string & state)
{
  try {
    transition_in_progress_ = true;

    if (obj_) {
      obj_->shutdown();
    }

    obj_.reset();
    obj_ = state_loader_.createSharedInstance(state);
    obj_->initialize(this->shared_from_this());

    if (state == "FaultMonitorMode") {
      current_state_ = functional_safety::SafetyState::FAULT_MONITOR;
    } else if (state == "SafeStopMode") {
      current_state_ = functional_safety::SafetyState::SAFE_STOP;
    } else if (state == "CompliantMode") {
      current_state_ = functional_safety::SafetyState::COMPLIANT_MODE;
    } else if (state == "TorqueLimitMode") {
      current_state_ = functional_safety::SafetyState::TORQUE_LIMIT_MODE;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown state/plugin name: %s", state.c_str());
      transition_in_progress_ = false;
      return false;
    }

    transition_in_progress_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "Changed state to: %s",
      state.c_str());

    return true;

  } catch (const std::exception & e) {
    transition_in_progress_ = false;
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to change state to %s: %s",
      state.c_str(),
      e.what());
    return false;
  }
}

bool StateMachine::request_state_change(
  functional_safety::SafetyState target_state,
  const std::string & reason)
{
  if (transition_in_progress_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Transition rejected: another transition is already in progress");
    return false;
  }

  if (!can_transition_to(target_state)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Transition rejected: %s -> %s | reason=%s",
      state_to_string(current_state_).c_str(),
      state_to_string(target_state).c_str(),
      reason.c_str());
    return false;
  }

  if (target_state == functional_safety::SafetyState::SAFE_STOP) {
    latch_fault(reason);
  }

  return change_state(state_to_plugin_name(target_state));
}

bool StateMachine::can_transition_to(functional_safety::SafetyState target_state) const
{
  if (safe_stop_latched_ &&
      target_state != functional_safety::SafetyState::SAFE_STOP) {
    return false;
  }

  if (target_state == current_state_) {
    return true;
  }

  switch (current_state_) {
    case functional_safety::SafetyState::FAULT_MONITOR:
      return
        target_state == functional_safety::SafetyState::COMPLIANT_MODE ||
        target_state == functional_safety::SafetyState::TORQUE_LIMIT_MODE ||
        target_state == functional_safety::SafetyState::SAFE_STOP;

    case functional_safety::SafetyState::COMPLIANT_MODE:
      return
        target_state == functional_safety::SafetyState::TORQUE_LIMIT_MODE ||
        target_state == functional_safety::SafetyState::SAFE_STOP ||
        target_state == functional_safety::SafetyState::FAULT_MONITOR;

    case functional_safety::SafetyState::TORQUE_LIMIT_MODE:
      return
        target_state == functional_safety::SafetyState::SAFE_STOP ||
        target_state == functional_safety::SafetyState::FAULT_MONITOR;

    case functional_safety::SafetyState::SAFE_STOP:
      return
        target_state == functional_safety::SafetyState::SAFE_STOP;

    default:
      return false;
  }
}

std::string StateMachine::state_to_plugin_name(functional_safety::SafetyState state) const
{
  switch (state) {
    case functional_safety::SafetyState::FAULT_MONITOR:
      return "FaultMonitorMode";
    case functional_safety::SafetyState::SAFE_STOP:
      return "SafeStopMode";
    case functional_safety::SafetyState::COMPLIANT_MODE:
      return "CompliantMode";
    case functional_safety::SafetyState::TORQUE_LIMIT_MODE:
      return "TorqueLimitMode";
    default:
      return "FaultMonitorMode";
  }
}

std::string StateMachine::state_to_string(functional_safety::SafetyState state) const
{
  switch (state) {
    case functional_safety::SafetyState::FAULT_MONITOR:
      return "FAULT_MONITOR";
    case functional_safety::SafetyState::SAFE_STOP:
      return "SAFE_STOP";
    case functional_safety::SafetyState::COMPLIANT_MODE:
      return "COMPLIANT_MODE";
    case functional_safety::SafetyState::TORQUE_LIMIT_MODE:
      return "TORQUE_LIMIT_MODE";
    default:
      return "UNKNOWN";
  }
}

void StateMachine::latch_fault(const std::string & reason)
{
  safe_stop_latched_ = true;
  fault_present_ = true;
  last_fault_reason_ = reason;

  RCLCPP_ERROR(
    this->get_logger(),
    "Fault latched | reason=%s",
    last_fault_reason_.c_str());
}

void StateMachine::clear_fault_state()
{
  safe_stop_latched_ = false;
  fault_present_ = false;
  last_fault_reason_ = "none";

  RCLCPP_INFO(this->get_logger(), "Fault state cleared");
}

void StateMachine::safe_stop_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    const bool ok = request_state_change(
      functional_safety::SafetyState::SAFE_STOP,
      "safe_stop_request");

    response->success = ok;
    response->message = ok ?
      "SAFE_STOP latched and activated" :
      "SAFE_STOP request rejected";
    return;
  }

  response->success = false;
  response->message =
    "SAFE_STOP cannot be cleared with SetBool(false). Use /reset_safety_request";
}

void StateMachine::compliant_mode_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    fault_present_ = true;

    const bool ok = request_state_change(
      functional_safety::SafetyState::COMPLIANT_MODE,
      "compliant_mode_request");

    response->success = ok;
    response->message = ok ?
      "COMPLIANT_MODE activated" :
      "COMPLIANT_MODE request rejected";
    return;
  }

  if (current_state_ == functional_safety::SafetyState::COMPLIANT_MODE) {
    const bool ok = request_state_change(
      functional_safety::SafetyState::FAULT_MONITOR,
      "compliant_mode_release");

    if (ok) {
      fault_present_ = false;
    }

    response->success = ok;
    response->message = ok ?
      "Returned to FAULT_MONITOR from COMPLIANT_MODE" :
      "Return to FAULT_MONITOR rejected";
    return;
  }

  response->success = true;
  response->message = "COMPLIANT_MODE already inactive";
}

void StateMachine::torque_limit_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    fault_present_ = true;

    const bool ok = request_state_change(
      functional_safety::SafetyState::TORQUE_LIMIT_MODE,
      "torque_limit_request");

    response->success = ok;
    response->message = ok ?
      "TORQUE_LIMIT_MODE activated" :
      "TORQUE_LIMIT_MODE request rejected";
    return;
  }

  if (current_state_ == functional_safety::SafetyState::TORQUE_LIMIT_MODE) {
    const bool ok = request_state_change(
      functional_safety::SafetyState::FAULT_MONITOR,
      "torque_limit_release");

    if (ok) {
      fault_present_ = false;
    }

    response->success = ok;
    response->message = ok ?
      "Returned to FAULT_MONITOR from TORQUE_LIMIT_MODE" :
      "Return to FAULT_MONITOR rejected";
    return;
  }

  response->success = true;
  response->message = "TORQUE_LIMIT_MODE already inactive";
}

void StateMachine::reset_safety_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!safe_stop_latched_ && !fault_present_) {
    response->success = true;
    response->message = "No latched fault to reset";
    return;
  }

  clear_fault_state();

  if (current_state_ != functional_safety::SafetyState::FAULT_MONITOR) {
    const bool ok = change_state("FaultMonitorMode");
    response->success = ok;
    response->message = ok ?
      "Safety reset completed, returned to FAULT_MONITOR" :
      "Fault cleared, but failed to return to FAULT_MONITOR";
    return;
  }

  response->success = true;
  response->message = "Safety reset completed";
}

}  // namespace functional_safety