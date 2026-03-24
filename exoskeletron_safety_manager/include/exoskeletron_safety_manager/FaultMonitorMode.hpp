#ifndef EXOSKELETRON_SAFETY_MANAGER_FAULT_MONITOR_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_FAULT_MONITOR_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/BridgeModeClient.hpp"

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

// FIX BUG 1/2: FaultMonitorMode ora eredita anche da BridgeModeClient
// per poter chiamare request_mode("nominal") nel suo initialize().
// Questo garantisce che all'ingresso in FAULT_MONITOR (sia da init che
// da reset) il bridge venga esplicitamente portato a "nominal".
class FaultMonitorMode : public SafetyTools, protected BridgeModeClient
{
public:
  FaultMonitorMode() = default;
  ~FaultMonitorMode() override = default;

  void initialize(const rclcpp::Node::SharedPtr & node) override;
  void stop() override;
  void pause() override;
  void resume() override;
  void shutdown() override;

  /**
   * set_safety_params(param):
   *   Per retrocompatibilità imposta tau_safe_stop_threshold_.
   */
  void set_safety_params(double param) override;

  /**
   * Legge tutte le soglie dai parametri ROS dichiarati sul nodo host.
   * Chiamato internamente da initialize().
   */
  void load_thresholds_from_params();

private:
  void bridge_status_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void watchdog_callback();

  // Valutazione ingresso (escalation)
  FaultResponseLevel evaluate_tau_entry(double tau_in) const;
  FaultResponseLevel evaluate_velocity_entry(double theta_dot) const;
  FaultResponseLevel max_level(FaultResponseLevel a, FaultResponseLevel b) const;
  bool is_more_severe(FaultResponseLevel candidate, FaultResponseLevel current) const;

  // Debounce ingresso
  void update_debounce_counters(FaultResponseLevel level);
  FaultResponseLevel debounced_entry_level() const;

  // Comunicazione verso StateMachine
  void request_compliant_mode();
  void request_torque_limit_mode();
  void request_safe_stop();
  bool call_bool_service(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client,
    const std::string & service_name,
    bool value);

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr bridge_status_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr compliant_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_limit_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr safe_stop_client_;

  // Soglie ingresso
  double tau_compliant_threshold_{2.0};
  double tau_torque_limit_threshold_{3.0};
  double tau_safe_stop_threshold_{4.0};
  double vel_compliant_threshold_{1.0};
  double vel_torque_limit_threshold_{2.0};
  double vel_safe_stop_threshold_{3.0};

  // Debounce ingresso
  int compliant_counter_{0};
  int torque_counter_{0};
  int stop_counter_{0};
  int compliant_count_threshold_{3};
  int torque_count_threshold_{3};
  int stop_count_threshold_{2};

  FaultResponseLevel last_requested_level_{FaultResponseLevel::NOMINAL};

  // Watchdog su /exo_bridge/status
  bool message_received_{false};
  bool watchdog_triggered_{false};
  rclcpp::Time last_msg_time_;
  double status_timeout_seconds_{0.5};

  // Grace period post-inizializzazione
  bool grace_period_active_{false};
  rclcpp::Time grace_period_end_;
  double grace_period_seconds_{2.0};
};

}  // namespace functional_safety

#endif