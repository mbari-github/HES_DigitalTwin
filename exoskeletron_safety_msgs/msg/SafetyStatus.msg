# Stato corrente della state machine
# Pubblicato su /safety_manager/status a ogni cambio di stato e periodicamente

builtin_interfaces/Time stamp

# Stato corrente come stringa e come ID numerico
# 0=FAULT_MONITOR 1=COMPLIANT_MODE 2=TORQUE_LIMIT_MODE 3=SAFE_STOP
string  state
int32   state_id

# Fault
bool    safe_stop_latched
bool    fault_present
string  last_fault_reason

# Contatori downgrade
int32   counter_exit_compliant
int32   counter_exit_torque_limit
int32   downgrade_threshold

# Progressione downgrade in percentuale [0.0, 100.0]
float64 downgrade_progress_compliant
float64 downgrade_progress_torque_limit

# Timestamp e motivo ultimo cambio di stato
builtin_interfaces/Time last_transition_time
string  last_transition_reason