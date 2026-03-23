#ifndef EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/safety_states.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "exoskeletron_safety_msgs/msg/safety_status.hpp"

namespace functional_safety
{

class StateMachine : public rclcpp::Node
{
public:
  StateMachine();
  void initialize();

private:
  // ── Transizioni ──────────────────────────────────────────────────
  bool change_state(const std::string & state, const std::string & reason = "");
  bool request_state_change(
    functional_safety::SafetyState target_state,
    const std::string & reason);

  bool can_transition_to(functional_safety::SafetyState target_state) const;
  std::string state_to_plugin_name(functional_safety::SafetyState state) const;
  std::string state_to_string(functional_safety::SafetyState state) const;
  int state_to_id(functional_safety::SafetyState state) const;

  // ── Fault latch ──────────────────────────────────────────────────
  void latch_fault(const std::string & reason);
  void clear_fault_state();

  // ── Service callbacks ────────────────────────────────────────────
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

  // ── Downgrade automatico ─────────────────────────────────────────
  void bridge_status_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void evaluate_downgrade(double tau_in, double theta_dot);
  void reset_downgrade_counters();

  // ── Publisher stato ──────────────────────────────────────────────
  void publish_status();
  void publish_status_string();

private:
  // ── Plugin ───────────────────────────────────────────────────────
  functional_safety::SafetyState current_state_{
    functional_safety::SafetyState::FAULT_MONITOR};

  pluginlib::ClassLoader<functional_safety::SafetyTools> state_loader_;
  std::shared_ptr<functional_safety::SafetyTools> obj_;

  // ── Services ─────────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr safe_stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr compliant_mode_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_limit_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_safety_service_;

  // ── Subscriptions ────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    bridge_status_sub_;

  // ── Publishers ───────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_string_pub_;
  rclcpp::Publisher<exoskeletron_safety_msgs::msg::SafetyStatus>::SharedPtr
    status_pub_;

  // Timer pubblicazione periodica
  rclcpp::TimerBase::SharedPtr status_timer_;

  // ── Fault state ──────────────────────────────────────────────────
  bool safe_stop_latched_      {false};
  bool fault_present_          {false};
  bool transition_in_progress_ {false};
  std::string last_fault_reason_      {"none"};
  std::string last_transition_reason_ {"init"};
  rclcpp::Time last_transition_time_;

  // ── Soglie downgrade ─────────────────────────────────────────────
  double tau_compliant_exit_    {1.5};
  double tau_torque_limit_exit_ {2.5};
  double vel_compliant_exit_    {0.8};
  double vel_torque_limit_exit_ {1.5};

  int downgrade_count_threshold_ {2000};
  int downgrade_negative_weight_ {5};

  int counter_exit_compliant_    {0};
  int counter_exit_torque_limit_ {0};
};

}  // namespace functional_safety
#endif