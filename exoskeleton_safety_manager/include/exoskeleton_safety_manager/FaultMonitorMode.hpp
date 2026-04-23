#ifndef EXOSKELETRON_SAFETY_MANAGER_FAULT_MONITOR_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_FAULT_MONITOR_MODE_HPP

#include "exoskeleton_safety_manager/SafetyTools.hpp"
#include "exoskeleton_safety_manager/BridgeModeClient.hpp"

#include <rclcpp/rclcpp.hpp>
#include <exoskeleton_safety_msgs/msg/float64_array_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace functional_safety
{

/**
 * FaultResponseLevel — escalation severity returned by the monitoring logic.
 *
 * Ordered by severity (NOMINAL < COMPLIANT < TORQUE_LIMIT < SAFE_STOP).
 * The integer values are used directly for comparison in is_more_severe().
 */
enum class FaultResponseLevel
{
  NOMINAL = 0,
  COMPLIANT = 1,
  TORQUE_LIMIT = 2,
  SAFE_STOP = 3
};

/**
 * FaultMonitorMode — the default (non-fault) plugin loaded at startup and after reset.
 *
 * This plugin monitors the bridge status and escalates to a more restrictive mode
 * if torque or velocity exceed the configured thresholds. It also inherits from
 * BridgeModeClient so that it can explicitly set the bridge to 'nominal' in its
 * initialize() — this ensures alignment between the state machine and the bridge
 * both at startup and after a reset from SAFE_STOP.
 *
 * Escalation logic
 * ----------------
 *   1. On each /exo_bridge/status message, the absolute values of tau_out and
 *      theta_dot are evaluated against three threshold pairs (compliant,
 *      torque_limit, safe_stop).
 *   2. Debounce counters prevent spurious escalations: the instantaneous level
 *      must be sustained for compliant_count_threshold / torque_count_threshold /
 *      stop_count_threshold consecutive samples before a request is sent.
 *   3. Escalation is monotone within an activation: once TORQUE_LIMIT has been
 *      requested, COMPLIANT is not re-requested (is_more_severe check).
 *
 * Watchdog
 * --------
 * A 100 ms wall timer checks that /exo_bridge/status messages arrive within
 * status_timeout_seconds. If the timeout fires after the first message has been
 * received (i.e. the bridge was alive and then died), SAFE_STOP is requested.
 *
 * Grace period
 * ------------
 * A grace_period_seconds window is given after initialize() during which
 * monitoring is suppressed. This prevents spurious escalations during system
 * startup before all nodes are publishing.
 *
 * Note on tau_out vs tau_raw
 * --------------------------
 * The escalation evaluates tau_out (data[5], post-clamp) because it represents
 * what the plant is actually experiencing. tau_raw (data[8], pre-clamp) is used
 * only by StateMachine for downgrade decisions.
 */
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

  /** Sets tau_safe_stop_threshold_ (kept for backwards compatibility). */
  void set_safety_params(double param) override;

  /** Reads all thresholds from ROS parameters declared on the host node. */
  void load_thresholds_from_params();

private:
  void bridge_status_callback(const exoskeleton_safety_msgs::msg::Float64ArrayStamped::SharedPtr msg);
  void watchdog_callback();

  // Escalation evaluation
  FaultResponseLevel evaluate_tau_entry(double tau_in) const;
  FaultResponseLevel evaluate_velocity_entry(double theta_dot) const;
  FaultResponseLevel max_level(FaultResponseLevel a, FaultResponseLevel b) const;
  bool is_more_severe(FaultResponseLevel candidate, FaultResponseLevel current) const;

  // Debounce
  void update_debounce_counters(FaultResponseLevel level);
  FaultResponseLevel debounced_entry_level() const;

  // Service calls toward StateMachine
  void request_compliant_mode();
  void request_torque_limit_mode();
  void request_safe_stop();
  bool call_bool_service(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client,
    const std::string & service_name,
    bool value);

private:
  rclcpp::Subscription<exoskeleton_safety_msgs::msg::Float64ArrayStamped>::SharedPtr bridge_status_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr compliant_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_limit_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr safe_stop_client_;

  // Escalation thresholds
  double tau_compliant_threshold_{2.0};
  double tau_torque_limit_threshold_{3.0};
  double tau_safe_stop_threshold_{4.0};
  double vel_compliant_threshold_{1.0};
  double vel_torque_limit_threshold_{2.0};
  double vel_safe_stop_threshold_{3.0};

  // Debounce counters and thresholds
  int compliant_counter_{0};
  int torque_counter_{0};
  int stop_counter_{0};
  int compliant_count_threshold_{3};
  int torque_count_threshold_{3};
  int stop_count_threshold_{2};

  FaultResponseLevel last_requested_level_{FaultResponseLevel::NOMINAL};

  // Watchdog on /exo_bridge/status
  bool message_received_{false};
  bool watchdog_triggered_{false};
  rclcpp::Time last_msg_time_;
  double status_timeout_seconds_{0.5};

  // Grace period after initialization
  bool grace_period_active_{false};
  rclcpp::Time grace_period_end_;
  double grace_period_seconds_{2.0};
};

}  // namespace functional_safety

#endif
