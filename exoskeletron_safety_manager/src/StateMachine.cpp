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

  try {
    obj_ = state_loader_.createSharedInstance("FaultMonitorMode");
    obj_->initialize(this->shared_from_this());
    RCLCPP_INFO(this->get_logger(), "Initial state: FAULT_MONITOR");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load initial plugin: %s", e.what());
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

  sensor_degraded_service_ = this->create_service<std_srvs::srv::SetBool>(
    "sensor_degraded_request",
    std::bind(
      &StateMachine::sensor_degraded_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void StateMachine::change_state(const std::string & state)
{
  try {
    if (obj_) {
      obj_->shutdown();
    }

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
    } else if (state == "SensorDegradedMode") {
      current_state_ = functional_safety::SafetyState::SENSOR_DEGRADED_MODE;
    }

    RCLCPP_INFO(this->get_logger(), "Changed state to: %s", state.c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to change state to %s: %s",
      state.c_str(),
      e.what());
  }
}

void StateMachine::safe_stop_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (current_state_ != functional_safety::SafetyState::SAFE_STOP) {
      change_state("SafeStopMode");
    }
    response->message = "SAFE_STOP activated";
  } else {
    if (current_state_ == functional_safety::SafetyState::SAFE_STOP) {
      change_state("FaultMonitorMode");
    }
    response->message = "Returned to FAULT_MONITOR from SAFE_STOP";
  }

  response->success = true;
}

void StateMachine::compliant_mode_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (current_state_ != functional_safety::SafetyState::COMPLIANT_MODE) {
      change_state("CompliantMode");
    }
    response->message = "COMPLIANT_MODE activated";
  } else {
    if (current_state_ == functional_safety::SafetyState::COMPLIANT_MODE) {
      change_state("FaultMonitorMode");
    }
    response->message = "Returned to FAULT_MONITOR from COMPLIANT_MODE";
  }

  response->success = true;
}

void StateMachine::torque_limit_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (current_state_ != functional_safety::SafetyState::TORQUE_LIMIT_MODE) {
      change_state("TorqueLimitMode");
    }
    response->message = "TORQUE_LIMIT_MODE activated";
  } else {
    if (current_state_ == functional_safety::SafetyState::TORQUE_LIMIT_MODE) {
      change_state("FaultMonitorMode");
    }
    response->message = "Returned to FAULT_MONITOR from TORQUE_LIMIT_MODE";
  }

  response->success = true;
}

void StateMachine::sensor_degraded_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (current_state_ != functional_safety::SafetyState::SENSOR_DEGRADED_MODE) {
      change_state("SensorDegradedMode");
    }
    response->message = "SENSOR_DEGRADED_MODE activated";
  } else {
    if (current_state_ == functional_safety::SafetyState::SENSOR_DEGRADED_MODE) {
      change_state("FaultMonitorMode");
    }
    response->message = "Returned to FAULT_MONITOR from SENSOR_DEGRADED_MODE";
  }

  response->success = true;
}

}  // namespace functional_safety