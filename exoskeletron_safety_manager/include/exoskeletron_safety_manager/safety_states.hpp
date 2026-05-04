#ifndef EXOSKELETRON_SAFETY_MANAGER_SAFETY_STATES_HPP
#define EXOSKELETRON_SAFETY_MANAGER_SAFETY_STATES_HPP

namespace functional_safety {

/**
 * SafetyState — enumeration of all possible states of the safety state machine.
 *
 * Allowed transitions (see StateMachine::can_transition_to):
 *
 *   FAULT_MONITOR  ──►  COMPLIANT_MODE
 *                  ──►  TORQUE_LIMIT_MODE
 *                  ──►  SAFE_STOP
 *
 *   COMPLIANT_MODE ──►  TORQUE_LIMIT_MODE
 *                  ──►  FAULT_MONITOR  (automatic downgrade)
 *                  ──►  SAFE_STOP
 *
 *   TORQUE_LIMIT   ──►  COMPLIANT_MODE (automatic downgrade)
 *                  ──►  FAULT_MONITOR  (automatic downgrade)
 *                  ──►  SAFE_STOP
 *
 *   SAFE_STOP      ──►  FAULT_MONITOR  (only via /reset_safety_request)
 *
 * SENSOR_DEGRADED_MODE is reserved for future use.
 */
enum class SafetyState {
    FAULT_MONITOR,       ///< Normal monitoring — no active fault, FaultMonitorMode plugin loaded
    SAFE_STOP,           ///< Latched emergency stop — bridge set to 'stop', requires manual reset
    COMPLIANT_MODE,      ///< Reduced torque and velocity limits — bridge set to 'compliant'
    TORQUE_LIMIT_MODE,   ///< Hard torque cap — bridge set to 'torque_limit'
    SENSOR_DEGRADED_MODE ///< Reserved — not yet implemented
};

}  // namespace functional_safety

#endif
