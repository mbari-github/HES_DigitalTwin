#ifndef EXOSKELETRON_SAFETY_MANAGER_FAULT_MONITOR_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_FAULT_MONITOR_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace functional_safety
{

enum class FaultResponseLevel
{
  NOMINAL = 0,
  COMPLIANT = 1,
  TORQUE_LIMIT = 2,
  SAFE_STOP = 3
};

class FaultMonitorMode : public SafetyTools
{
public:
  FaultMonitorMode() = default;
  ~FaultMonitorMode() override = default;

  void initialize(const rclcpp::Node::SharedPtr & node) override;
  void stop() override;
  void pause() override;
  void resume() override;
  void shutdown() override;
  void set_safety_params(double param) override;

private:
  void bridge_status_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void watchdog_callback();

  FaultResponseLevel evaluate_tau(double tau_in) const;
  FaultResponseLevel evaluate_velocity(double theta_dot) const;
  FaultResponseLevel max_level(FaultResponseLevel a, FaultResponseLevel b) const;
  bool is_more_severe(FaultResponseLevel candidate, FaultResponseLevel current) const;
  void update_debounce_counters(FaultResponseLevel level);
  FaultResponseLevel debounced_level() const;

  void request_compliant_mode();
  void request_torque_limit_mode();
  void request_safe_stop();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr bridge_status_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr compliant_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_limit_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr safe_stop_client_;

  double tau_compliant_threshold_{0.75};
  double tau_torque_limit_threshold_{1.5};
  double tau_safe_stop_threshold_{3.0};

  double vel_compliant_threshold_{1.0};
  double vel_torque_limit_threshold_{2.0};
  double vel_safe_stop_threshold_{3.0};

  FaultResponseLevel last_requested_level_{FaultResponseLevel::NOMINAL};

  int compliant_counter_{0};
  int torque_counter_{0};
  int stop_counter_{0};

  const int compliant_count_threshold_{3};
  const int torque_count_threshold_{3};
  const int stop_count_threshold_{2};

  bool message_received_{false};
  bool watchdog_triggered_{false};
  rclcpp::Time last_msg_time_;
  double status_timeout_seconds_{0.5};
};

}  // namespace functional_safety

#endif