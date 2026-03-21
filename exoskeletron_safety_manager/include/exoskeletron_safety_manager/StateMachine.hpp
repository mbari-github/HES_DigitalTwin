#ifndef EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/safety_states.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace functional_safety
{

class StateMachine : public rclcpp::Node
{
public:
  StateMachine();
  void initialize();

private:
  bool change_state(const std::string & state);
  bool request_state_change(
    functional_safety::SafetyState target_state,
    const std::string & reason);

  bool can_transition_to(functional_safety::SafetyState target_state) const;
  std::string state_to_plugin_name(functional_safety::SafetyState state) const;
  std::string state_to_string(functional_safety::SafetyState state) const;

  void latch_fault(const std::string & reason);
  void clear_fault_state();

  void safe_stop_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void compliant_mode_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void torque_limit_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void reset_safety_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  functional_safety::SafetyState current_state_{functional_safety::SafetyState::FAULT_MONITOR};

  pluginlib::ClassLoader<functional_safety::SafetyTools> state_loader_;
  std::shared_ptr<functional_safety::SafetyTools> obj_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr safe_stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr compliant_mode_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_limit_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_safety_service_;

  bool safe_stop_latched_{false};
  bool fault_present_{false};
  bool transition_in_progress_{false};
  std::string last_fault_reason_{"none"};
};

}  // namespace functional_safety

#endif