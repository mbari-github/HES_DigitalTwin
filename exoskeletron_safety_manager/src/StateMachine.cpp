#include "exoskeletron_safety_manager/StateMachine.hpp"

namespace functional_safety
{

// ════════════════════════════════════════════════════════════════════
// Costruttore
// ════════════════════════════════════════════════════════════════════

StateMachine::StateMachine()
: Node("state_machine"),
  state_loader_("exoskeletron_safety_manager", "functional_safety::SafetyTools")
{
  this->declare_parameter("fault_monitor.enable_automatic_downgrade", false);
  this->declare_parameter("fault_monitor.tau_compliant_exit",        1.5);
  this->declare_parameter("fault_monitor.tau_torque_limit_exit",     2.5);
  this->declare_parameter("fault_monitor.vel_compliant_exit",        0.8);
  this->declare_parameter("fault_monitor.vel_torque_limit_exit",     1.5);
  this->declare_parameter("fault_monitor.downgrade_count_threshold", 2000);
  this->declare_parameter("fault_monitor.downgrade_negative_weight", 5);

  enable_automatic_downgrade_ =
    this->get_parameter("fault_monitor.enable_automatic_downgrade").as_bool();
  tau_compliant_exit_    = this->get_parameter("fault_monitor.tau_compliant_exit").as_double();
  tau_torque_limit_exit_ = this->get_parameter("fault_monitor.tau_torque_limit_exit").as_double();
  vel_compliant_exit_    = this->get_parameter("fault_monitor.vel_compliant_exit").as_double();
  vel_torque_limit_exit_ = this->get_parameter("fault_monitor.vel_torque_limit_exit").as_double();
  downgrade_count_threshold_ =
    this->get_parameter("fault_monitor.downgrade_count_threshold").as_int();
  downgrade_negative_weight_ =
    this->get_parameter("fault_monitor.downgrade_negative_weight").as_int();

  last_transition_time_ = this->now();
}

// ════════════════════════════════════════════════════════════════════
// Initialize
// ════════════════════════════════════════════════════════════════════

void StateMachine::initialize()
{
  current_state_ = functional_safety::SafetyState::FAULT_MONITOR;

  if (!change_state("FaultMonitorMode", "init")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize in FAULT_MONITOR");
  }

  // ── Services ─────────────────────────────────────────────────────
  safe_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
    "safe_stop_request",
    std::bind(&StateMachine::safe_stop_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  compliant_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
    "compliant_mode_request",
    std::bind(&StateMachine::compliant_mode_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  torque_limit_service_ = this->create_service<std_srvs::srv::SetBool>(
    "torque_limit_request",
    std::bind(&StateMachine::torque_limit_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  reset_safety_service_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_safety_request",
    std::bind(&StateMachine::reset_safety_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // ── Subscription per downgrade ───────────────────────────────────
  bridge_status_sub_ =
    this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/exo_bridge/status", 10,
      std::bind(&StateMachine::bridge_status_callback, this,
        std::placeholders::_1));

  // ── Publishers ───────────────────────────────────────────────────
  state_string_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/safety_manager/state", 10);

  status_pub_ = this->create_publisher<exoskeletron_safety_msgs::msg::SafetyStatus>(
    "/safety_manager/status", 10);

  // Pubblica a 10Hz anche senza cambi di stato
  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&StateMachine::publish_status, this));

  RCLCPP_INFO(this->get_logger(),
    "StateMachine initialized | state_topic=/safety_manager/state\n"
    "  automatic_downgrade     = %s\n"
    "  tau_compliant_exit      = %.2f Nm\n"
    "  tau_torque_limit_exit   = %.2f Nm\n"
    "  vel_compliant_exit      = %.2f rad/s\n"
    "  vel_torque_limit_exit   = %.2f rad/s\n"
    "  downgrade_threshold     = %d samples (%.1fs @ 200Hz)\n"
    "  downgrade_neg_weight    = %d",
    enable_automatic_downgrade_ ? "ENABLED" : "DISABLED",
    tau_compliant_exit_, tau_torque_limit_exit_,
    vel_compliant_exit_, vel_torque_limit_exit_,
    downgrade_count_threshold_,
    downgrade_count_threshold_ / 200.0,
    downgrade_negative_weight_);
}

// ════════════════════════════════════════════════════════════════════
// Publisher stato
// ════════════════════════════════════════════════════════════════════

void StateMachine::publish_status()
{
  if (!status_pub_) return;

  exoskeletron_safety_msgs::msg::SafetyStatus msg;
  msg.stamp             = this->now();
  msg.state             = state_to_string(current_state_);
  msg.state_id          = state_to_id(current_state_);
  msg.safe_stop_latched = safe_stop_latched_;
  msg.fault_present     = fault_present_;
  msg.last_fault_reason = last_fault_reason_;

  msg.counter_exit_compliant    = counter_exit_compliant_;
  msg.counter_exit_torque_limit = counter_exit_torque_limit_;
  msg.downgrade_threshold       = downgrade_count_threshold_;

  msg.downgrade_progress_compliant = downgrade_count_threshold_ > 0 ?
    100.0 * counter_exit_compliant_ / downgrade_count_threshold_ : 0.0;

  msg.downgrade_progress_torque_limit = downgrade_count_threshold_ > 0 ?
    100.0 * counter_exit_torque_limit_ / downgrade_count_threshold_ : 0.0;

  msg.last_transition_time   = last_transition_time_;
  msg.last_transition_reason = last_transition_reason_;

  status_pub_->publish(msg);
  publish_status_string();
}

void StateMachine::publish_status_string()
{
  if (!state_string_pub_) return;
  std_msgs::msg::String msg;
  msg.data = state_to_string(current_state_);
  state_string_pub_->publish(msg);
}

// ════════════════════════════════════════════════════════════════════
// Downgrade automatico
// ════════════════════════════════════════════════════════════════════
//
// Downgrade: usa tau_out (data[5], coppia post-clamp) e theta_dot (data[3]).
//
// Perché tau_out e non tau_raw:
// Quando il bridge è in torque_limit, la coppia è clampata a override_tau_limit.
// Il controller PD in closed-loop reagisce alla coppia clampata producendo una
// tau_raw sempre più alta (il plant non si muove abbastanza → errore cresce →
// tau cresce). Quindi tau_raw resta elevata per TUTTA la durata della mode,
// anche dopo la rimozione del fault, rendendo il downgrade impossibile.
//
// tau_out riflette lo stato REALE del sistema: se il fault è stato rimosso e
// il plant converge, tau_out scende naturalmente. Se il fault è ancora attivo,
// tau_out resta al limite del clamp (= override_tau_limit), e il downgrade
// avviene solo se override_tau_limit < tau_torque_limit_exit, che è vero
// nella configurazione attuale (2.0 < 2.5 Nm).
//
// NOTA: il downgrade con tau_out funziona correttamente SOLO se le soglie
// di uscita sono coerenti con i limiti di coppia del bridge:
//   override_tau_limit (2.0) < tau_torque_limit_exit (2.5)  ✓
//   compliant_tau_limit (4.0) > tau_compliant_exit (1.5)    ✗ ← attenzione!
//
// Per il compliant mode: compliant_tau_limit (4.0) è SOPRA tau_compliant_exit
// (1.5), quindi il downgrade dipende dal fatto che il controller converga e
// produca meno di 1.5 Nm dopo la rimozione del fault. Se il fault è ancora
// attivo, la tau_out può restare sopra soglia e il downgrade non avviene.
// Questo è il comportamento corretto: con fault attivo non si deve fare
// downgrade.
//
// Layout dello status:
//   data[0] = is_stop          data[4] = theta_hold
//   data[1] = is_limited       data[5] = tau_out (post-clamp)
//   data[2] = theta            data[6] = tau_ext_theta
//   data[3] = theta_dot        data[7] = mode_id
//   data[8] = tau_raw (pre-clamp, disponibile per diagnostica)

void StateMachine::bridge_status_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 9) return;

  // Se il downgrade automatico è disabilitato, non valutare.
  // Le transizioni verso stati meno restrittivi avverranno solo
  // tramite i servizi ROS manuali (/compliant_mode_request data:=false,
  // /torque_limit_request data:=false, /reset_safety_request).
  if (!enable_automatic_downgrade_) return;

  if (current_state_ != functional_safety::SafetyState::COMPLIANT_MODE &&
      current_state_ != functional_safety::SafetyState::TORQUE_LIMIT_MODE) {
    reset_downgrade_counters();
    return;
  }

  if (safe_stop_latched_) return;

  const double theta_dot = msg->data[3];
  const double tau_out   = msg->data[5];

  evaluate_downgrade(tau_out, theta_dot);
}

void StateMachine::evaluate_downgrade(double tau_in, double theta_dot)
{
  const double abs_tau = std::abs(tau_in);
  const double abs_vel = std::abs(theta_dot);

  // ── Da TORQUE_LIMIT ──────────────────────────────────────────────
  if (current_state_ == functional_safety::SafetyState::TORQUE_LIMIT_MODE) {
    const bool positive = (abs_tau < tau_torque_limit_exit_) &&
                          (abs_vel < vel_torque_limit_exit_);

    if (positive) {
      counter_exit_torque_limit_ = std::min(
        counter_exit_torque_limit_ + 1,
        downgrade_count_threshold_);
    } else {
      counter_exit_torque_limit_ = std::max(
        counter_exit_torque_limit_ - downgrade_negative_weight_, 0);
    }

    if (counter_exit_torque_limit_ % 200 == 0 &&
        counter_exit_torque_limit_ > 0) {
      RCLCPP_INFO(this->get_logger(),
        "Downgrade progress [TORQUE_LIMIT]: %d/%d (%.1f%%)",
        counter_exit_torque_limit_,
        downgrade_count_threshold_,
        100.0 * counter_exit_torque_limit_ / downgrade_count_threshold_);
    }

    if (counter_exit_torque_limit_ >= downgrade_count_threshold_) {
      counter_exit_torque_limit_ = 0;
      counter_exit_compliant_    = 0;

      if (abs_tau < tau_compliant_exit_ && abs_vel < vel_compliant_exit_) {
        RCLCPP_INFO(this->get_logger(),
          "Downgrade: TORQUE_LIMIT -> FAULT_MONITOR | tau_raw=%.3f vel=%.3f",
          tau_in, theta_dot);
        fault_present_ = false;
        // FIX BUG 3: usa request_state_change per passare per la matrice
        // di transizione e i check su safe_stop_latched / transition_in_progress
        request_state_change(
          functional_safety::SafetyState::FAULT_MONITOR,
          "downgrade_torque_limit_to_nominal");
      } else {
        RCLCPP_INFO(this->get_logger(),
          "Downgrade: TORQUE_LIMIT -> COMPLIANT | tau_raw=%.3f vel=%.3f",
          tau_in, theta_dot);
        // FIX BUG 3: usa request_state_change
        request_state_change(
          functional_safety::SafetyState::COMPLIANT_MODE,
          "downgrade_torque_limit_to_compliant");
      }
    }
    return;
  }

  // ── Da COMPLIANT ─────────────────────────────────────────────────
  if (current_state_ == functional_safety::SafetyState::COMPLIANT_MODE) {
    const bool positive = (abs_tau < tau_compliant_exit_) &&
                          (abs_vel < vel_compliant_exit_);

    if (positive) {
      counter_exit_compliant_ = std::min(
        counter_exit_compliant_ + 1,
        downgrade_count_threshold_);
    } else {
      counter_exit_compliant_ = std::max(
        counter_exit_compliant_ - downgrade_negative_weight_, 0);
    }

    if (counter_exit_compliant_ % 200 == 0 &&
        counter_exit_compliant_ > 0) {
      RCLCPP_INFO(this->get_logger(),
        "Downgrade progress [COMPLIANT]: %d/%d (%.1f%%)",
        counter_exit_compliant_,
        downgrade_count_threshold_,
        100.0 * counter_exit_compliant_ / downgrade_count_threshold_);
    }

    if (counter_exit_compliant_ >= downgrade_count_threshold_) {
      counter_exit_compliant_ = 0;
      RCLCPP_INFO(this->get_logger(),
        "Downgrade: COMPLIANT -> FAULT_MONITOR | tau_raw=%.3f vel=%.3f",
        tau_in, theta_dot);
      fault_present_ = false;
      // FIX BUG 3: usa request_state_change
      request_state_change(
        functional_safety::SafetyState::FAULT_MONITOR,
        "downgrade_compliant_to_nominal");
    }
  }
}

void StateMachine::reset_downgrade_counters()
{
  counter_exit_compliant_    = 0;
  counter_exit_torque_limit_ = 0;
}

// ════════════════════════════════════════════════════════════════════
// Transizioni
// ════════════════════════════════════════════════════════════════════

bool StateMachine::change_state(const std::string & state, const std::string & reason)
{
  try {
    transition_in_progress_ = true;

    // FIX BUG 2: NON chiamare shutdown() sul plugin uscente.
    // Il vecchio plugin faceva request_mode("nominal") nel suo shutdown(),
    // creando un transitorio pericoloso (bridge brevemente in "nominal"
    // prima che il nuovo plugin imposti il modo corretto).
    // Ora il ciclo di vita è: reset il vecchio → crea il nuovo →
    // il nuovo imposta il bridge nel suo initialize().
    if (obj_) {
      obj_->stop();
      obj_.reset();
    }

    obj_ = state_loader_.createSharedInstance(state);
    obj_->initialize(this->shared_from_this());

    if      (state == "FaultMonitorMode") current_state_ = functional_safety::SafetyState::FAULT_MONITOR;
    else if (state == "SafeStopMode")     current_state_ = functional_safety::SafetyState::SAFE_STOP;
    else if (state == "CompliantMode")    current_state_ = functional_safety::SafetyState::COMPLIANT_MODE;
    else if (state == "TorqueLimitMode")  current_state_ = functional_safety::SafetyState::TORQUE_LIMIT_MODE;
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown state: %s", state.c_str());
      transition_in_progress_ = false;
      return false;
    }

    last_transition_time_   = this->now();
    last_transition_reason_ = reason.empty() ?
      "unspecified" : reason;
    transition_in_progress_ = false;

    RCLCPP_INFO(this->get_logger(),
      "State -> %s | reason=%s",
      state_to_string(current_state_).c_str(),
      last_transition_reason_.c_str());

    publish_status();

    return true;

  } catch (const std::exception & e) {
    transition_in_progress_ = false;
    RCLCPP_ERROR(this->get_logger(),
      "Failed to change state to %s: %s", state.c_str(), e.what());
    return false;
  }
}

bool StateMachine::request_state_change(
  functional_safety::SafetyState target_state,
  const std::string & reason)
{
  if (transition_in_progress_) {
    RCLCPP_WARN(this->get_logger(), "Transition rejected: already in progress");
    return false;
  }

  if (!can_transition_to(target_state)) {
    RCLCPP_WARN(this->get_logger(),
      "Transition rejected: %s -> %s | reason=%s",
      state_to_string(current_state_).c_str(),
      state_to_string(target_state).c_str(),
      reason.c_str());
    return false;
  }

  if (target_state == functional_safety::SafetyState::SAFE_STOP) {
    latch_fault(reason);
  }

  reset_downgrade_counters();
  return change_state(state_to_plugin_name(target_state), reason);
}

// FIX BUG 1: SAFE_STOP ora permette la transizione verso FAULT_MONITOR.
// La guardia safe_stop_latched_ impedisce comunque la transizione finché
// il latch non viene esplicitamente rilasciato da clear_fault_state()
// (chiamato da reset_safety_callback prima di request_state_change).
// Questo permette di usare request_state_change anche per il reset,
// eliminando il bypass diretto di change_state.
bool StateMachine::can_transition_to(
  functional_safety::SafetyState target_state) const
{
  if (safe_stop_latched_ &&
      target_state != functional_safety::SafetyState::SAFE_STOP) return false;

  if (target_state == current_state_) return true;

  switch (current_state_) {
    case functional_safety::SafetyState::FAULT_MONITOR:
      return target_state == functional_safety::SafetyState::COMPLIANT_MODE    ||
             target_state == functional_safety::SafetyState::TORQUE_LIMIT_MODE ||
             target_state == functional_safety::SafetyState::SAFE_STOP;

    case functional_safety::SafetyState::COMPLIANT_MODE:
      return target_state == functional_safety::SafetyState::TORQUE_LIMIT_MODE ||
             target_state == functional_safety::SafetyState::SAFE_STOP         ||
             target_state == functional_safety::SafetyState::FAULT_MONITOR;

    case functional_safety::SafetyState::TORQUE_LIMIT_MODE:
      return target_state == functional_safety::SafetyState::SAFE_STOP         ||
             target_state == functional_safety::SafetyState::FAULT_MONITOR     ||
             target_state == functional_safety::SafetyState::COMPLIANT_MODE;

    // FIX BUG 1: SAFE_STOP → FAULT_MONITOR ora è ammessa nella matrice.
    // La guardia safe_stop_latched_ (sopra) previene transizioni accidentali:
    // solo dopo clear_fault_state() il latch viene rilasciato e la
    // transizione diventa possibile.
    case functional_safety::SafetyState::SAFE_STOP:
      return target_state == functional_safety::SafetyState::SAFE_STOP    ||
             target_state == functional_safety::SafetyState::FAULT_MONITOR;

    default: return false;
  }
}

std::string StateMachine::state_to_plugin_name(
  functional_safety::SafetyState state) const
{
  switch (state) {
    case functional_safety::SafetyState::FAULT_MONITOR:    return "FaultMonitorMode";
    case functional_safety::SafetyState::SAFE_STOP:        return "SafeStopMode";
    case functional_safety::SafetyState::COMPLIANT_MODE:   return "CompliantMode";
    case functional_safety::SafetyState::TORQUE_LIMIT_MODE:return "TorqueLimitMode";
    default: return "FaultMonitorMode";
  }
}

std::string StateMachine::state_to_string(
  functional_safety::SafetyState state) const
{
  switch (state) {
    case functional_safety::SafetyState::FAULT_MONITOR:    return "FAULT_MONITOR";
    case functional_safety::SafetyState::SAFE_STOP:        return "SAFE_STOP";
    case functional_safety::SafetyState::COMPLIANT_MODE:   return "COMPLIANT_MODE";
    case functional_safety::SafetyState::TORQUE_LIMIT_MODE:return "TORQUE_LIMIT_MODE";
    default: return "UNKNOWN";
  }
}

int StateMachine::state_to_id(functional_safety::SafetyState state) const
{
  switch (state) {
    case functional_safety::SafetyState::FAULT_MONITOR:    return 0;
    case functional_safety::SafetyState::COMPLIANT_MODE:   return 1;
    case functional_safety::SafetyState::TORQUE_LIMIT_MODE:return 2;
    case functional_safety::SafetyState::SAFE_STOP:        return 3;
    default: return -1;
  }
}

// ════════════════════════════════════════════════════════════════════
// Fault latch
// ════════════════════════════════════════════════════════════════════

void StateMachine::latch_fault(const std::string & reason)
{
  safe_stop_latched_ = true;
  fault_present_     = true;
  last_fault_reason_ = reason;
  RCLCPP_ERROR(this->get_logger(),
    "Fault latched | reason=%s", reason.c_str());
}

void StateMachine::clear_fault_state()
{
  safe_stop_latched_ = false;
  fault_present_     = false;
  last_fault_reason_ = "none";
  reset_downgrade_counters();
  RCLCPP_INFO(this->get_logger(), "Fault state cleared");
}

// ════════════════════════════════════════════════════════════════════
// Service callbacks
// ════════════════════════════════════════════════════════════════════

void StateMachine::safe_stop_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    const bool ok = request_state_change(
      functional_safety::SafetyState::SAFE_STOP, "safe_stop_request");
    response->success = ok;
    response->message = ok ?
      "SAFE_STOP latched and activated" : "SAFE_STOP request rejected";
    return;
  }
  response->success = false;
  response->message = "Use /reset_safety_request to clear SAFE_STOP";
}

void StateMachine::compliant_mode_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    fault_present_ = true;
    const bool ok = request_state_change(
      functional_safety::SafetyState::COMPLIANT_MODE, "compliant_mode_request");
    response->success = ok;
    response->message = ok ?
      "COMPLIANT_MODE activated" : "COMPLIANT_MODE rejected";
    return;
  }

  if (current_state_ == functional_safety::SafetyState::COMPLIANT_MODE) {
    const bool ok = request_state_change(
      functional_safety::SafetyState::FAULT_MONITOR, "compliant_mode_release");
    if (ok) fault_present_ = false;
    response->success = ok;
    response->message = ok ?
      "Returned to FAULT_MONITOR from COMPLIANT_MODE" : "Return rejected";
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
      functional_safety::SafetyState::TORQUE_LIMIT_MODE, "torque_limit_request");
    response->success = ok;
    response->message = ok ?
      "TORQUE_LIMIT_MODE activated" : "TORQUE_LIMIT_MODE rejected";
    return;
  }

  if (current_state_ == functional_safety::SafetyState::TORQUE_LIMIT_MODE) {
    const bool ok = request_state_change(
      functional_safety::SafetyState::FAULT_MONITOR, "torque_limit_release");
    if (ok) fault_present_ = false;
    response->success = ok;
    response->message = ok ?
      "Returned to FAULT_MONITOR from TORQUE_LIMIT_MODE" : "Return rejected";
    return;
  }

  response->success = true;
  response->message = "TORQUE_LIMIT_MODE already inactive";
}

// FIX BUG 1/4: reset_safety ora passa per request_state_change.
// La sequenza è:
//   1. clear_fault_state() → rilascia safe_stop_latched_
//   2. request_state_change(FAULT_MONITOR) → passa per can_transition_to
//      (che ora permette SAFE_STOP → FAULT_MONITOR quando il latch è rilasciato)
//   3. change_state carica FaultMonitorMode, il cui initialize() NON tocca
//      il bridge (il bridge viene portato a "nominal" dal nuovo plugin
//      FaultMonitorMode tramite request_mode, vedi sotto)
//
// Nota: il FaultMonitorMode nel suo initialize() ora chiama
// request_mode("nominal") per assicurarsi che il bridge sia in nominal
// all'ingresso in FAULT_MONITOR. Questo risolve il coordinamento
// bridge/state_machine.
void StateMachine::reset_safety_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!safe_stop_latched_ && !fault_present_) {
    response->success = true;
    response->message = "No latched fault to reset";
    return;
  }

  // Rilascia il latch PRIMA della transizione, altrimenti
  // can_transition_to blocca SAFE_STOP → FAULT_MONITOR
  clear_fault_state();

  // Ora la transizione è possibile
  const bool ok = request_state_change(
    functional_safety::SafetyState::FAULT_MONITOR, "reset");
  response->success = ok;
  response->message = ok ?
    "Safety reset completed, returned to FAULT_MONITOR" :
    "Fault cleared but failed to return to FAULT_MONITOR";
}

}  // namespace functional_safety