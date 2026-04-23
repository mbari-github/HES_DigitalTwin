#ifndef EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "exoskeleton_safety_manager/SafetyTools.hpp"
#include "exoskeleton_safety_manager/safety_states.hpp"

#include "std_msgs/msg/string.hpp"
#include "exoskeleton_safety_msgs/msg/float64_array_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "exoskeleton_safety_msgs/msg/safety_status.hpp"

namespace functional_safety
{

/**
 * StateMachine — ROS2 node that manages the safety state of the exoskeleton.
 *
 * Architecture
 * ------------
 * The state machine is the central supervisor of the system. It loads safety
 * mode plugins (SafeStopMode, CompliantMode, TorqueLimitMode, FaultMonitorMode)
 * via pluginlib and orchestrates transitions between them.
 *
 * Each plugin is responsible for:
 *   - Setting the bridge to the correct operating mode in its initialize().
 *   - NOT changing the bridge mode in shutdown() (see SafetyTools.hpp).
 *
 * State transitions
 * -----------------
 * See safety_states.hpp for the full transition graph.
 * Transitions are always guarded by can_transition_to().
 *
 * Bridge supervision
 * ------------------
 * The state machine subscribes to /exo_bridge/status and performs two checks
 * at every tick (10 Hz via the status timer):
 *
 *   1. Liveness: if the bridge status has not been received within
 *      bridge_liveness_timeout_sec, the system transitions to SAFE_STOP.
 *      A grace period of bridge_liveness_grace_sec is given at startup.
 *
 *   2. Mode coherence: if the bridge reports a mode_id (data[7]) that does
 *      not match the expected ID for the current state machine state for
 *      more than mode_mismatch_threshold_ consecutive cycles, the system
 *      transitions to SAFE_STOP. The mismatch counter is reset on every
 *      state transition to account for the 1-2 cycle delay before the
 *      bridge updates its reported mode.
 *
 * Automatic downgrade
 * -------------------
 * When enable_automatic_downgrade is true, the state machine monitors tau_out
 * and theta_dot from /exo_bridge/status and uses a hysteresis counter to
 * automatically step down from TORQUE_LIMIT → COMPLIANT → FAULT_MONITOR
 * once the system has been operating below the exit thresholds for
 * downgrade_count_threshold consecutive samples.
 *
 * ROS2 Parameters
 * ---------------
 *   fault_monitor.enable_automatic_downgrade  (bool,  default false)
 *   fault_monitor.tau_compliant_exit          (float, default 1.5 Nm)
 *   fault_monitor.tau_torque_limit_exit       (float, default 2.5 Nm)
 *   fault_monitor.vel_compliant_exit          (float, default 0.8 rad/s)
 *   fault_monitor.vel_torque_limit_exit       (float, default 1.5 rad/s)
 *   fault_monitor.downgrade_count_threshold   (int,   default 2000 samples)
 *   fault_monitor.downgrade_negative_weight   (int,   default 5)
 *   bridge_liveness_timeout_sec               (float, default 0.5 s)
 *   bridge_liveness_grace_sec                 (float, default 3.0 s)
 *
 * Exposed services
 * ----------------
 *   /safe_stop_request       std_srvs/SetBool   — latch SAFE_STOP (data=true)
 *   /compliant_mode_request  std_srvs/SetBool   — enter/exit COMPLIANT_MODE
 *   /torque_limit_request    std_srvs/SetBool   — enter/exit TORQUE_LIMIT_MODE
 *   /reset_safety_request    std_srvs/Trigger   — clear latched fault, return to FAULT_MONITOR
 *
 * Published topics
 * ----------------
 *   /safety_manager/state    std_msgs/String            — current state name
 *   /safety_manager/status   SafetyStatus               — full diagnostic status
 */
class StateMachine : public rclcpp::Node
{
public:
  StateMachine();
  void initialize();

private:
  // ── State transitions ─────────────────────────────────────────────
  bool change_state(const std::string & state, const std::string & reason = "");
  bool request_state_change(
    functional_safety::SafetyState target_state,
    const std::string & reason);

  bool can_transition_to(functional_safety::SafetyState target_state) const;
  std::string state_to_plugin_name(functional_safety::SafetyState state) const;
  std::string state_to_string(functional_safety::SafetyState state) const;
  int state_to_id(functional_safety::SafetyState state) const;

  // ── Fault latch ───────────────────────────────────────────────────
  void latch_fault(const std::string & reason);
  void clear_fault_state();

  // ── Service callbacks ─────────────────────────────────────────────
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

  // ── Automatic downgrade ───────────────────────────────────────────
  void bridge_status_callback(
    const exoskeleton_safety_msgs::msg::Float64ArrayStamped::SharedPtr msg);

  void evaluate_downgrade(double tau_in, double theta_dot);
  void reset_downgrade_counters();

  // ── Bridge supervision ────────────────────────────────────────────
  int expected_bridge_mode_id() const;
  void check_bridge_liveness();

  // ── Status publishing ─────────────────────────────────────────────
  void publish_status();
  void publish_status_string();

private:
  // ── Plugin management ─────────────────────────────────────────────
  functional_safety::SafetyState current_state_{
    functional_safety::SafetyState::FAULT_MONITOR};

  pluginlib::ClassLoader<functional_safety::SafetyTools> state_loader_;
  std::shared_ptr<functional_safety::SafetyTools> obj_;

  // ── ROS services ─────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr safe_stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr compliant_mode_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_limit_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_safety_service_;

  // ── Subscriptions ─────────────────────────────────────────────────
  rclcpp::Subscription<exoskeleton_safety_msgs::msg::Float64ArrayStamped>::SharedPtr
    bridge_status_sub_;

  // ── Publishers ────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_string_pub_;
  rclcpp::Publisher<exoskeleton_safety_msgs::msg::SafetyStatus>::SharedPtr
    status_pub_;

  // Status/liveness timer (10 Hz)
  rclcpp::TimerBase::SharedPtr status_timer_;

  // ── Fault state ───────────────────────────────────────────────────
  bool safe_stop_latched_      {false};
  bool fault_present_          {false};
  bool transition_in_progress_ {false};
  std::string last_fault_reason_      {"none"};
  std::string last_transition_reason_ {"init"};
  rclcpp::Time last_transition_time_;

  // ── Automatic downgrade parameters ───────────────────────────────
  bool enable_automatic_downgrade_ {false};

  double tau_compliant_exit_    {1.5};
  double tau_torque_limit_exit_ {2.5};
  double vel_compliant_exit_    {0.8};
  double vel_torque_limit_exit_ {1.5};

  int downgrade_count_threshold_ {2000};
  int downgrade_negative_weight_ {5};

  int counter_exit_compliant_    {0};
  int counter_exit_torque_limit_ {0};

  // ── Bridge supervision state ──────────────────────────────────────
  //
  // Liveness: updated on every /exo_bridge/status reception.
  // Checked inside publish_status() (already at 10 Hz) — no extra timer needed.
  // If the message age exceeds bridge_liveness_timeout_, SAFE_STOP is requested.
  //
  // Mode coherence: counts consecutive cycles where the bridge's reported
  // mode_id (data[7]) does not match the expected ID for the current SM state.
  // If the count exceeds mode_mismatch_threshold_, SAFE_STOP is requested.
  // The counter is reset to 0 on every state transition to absorb the 1-2 cycle
  // lag before the bridge updates its reported mode.

  rclcpp::Time last_bridge_status_time_;
  bool bridge_ever_seen_       {false};
  bool bridge_alive_           {false};
  bool bridge_mode_coherent_   {true};

  double bridge_liveness_timeout_ {0.5};
  double bridge_liveness_grace_   {3.0};

  int mode_mismatch_count_     {0};
  static constexpr int mode_mismatch_threshold_ = 10;
};

}  // namespace functional_safety
#endif
