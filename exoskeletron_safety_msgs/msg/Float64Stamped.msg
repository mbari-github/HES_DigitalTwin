# Scalar float with timestamp — replaces std_msgs/Float64
# for all topics that require time synchronization.
#
# Usage: torque_raw, tau_ext_theta, tau_ext_theta_override, etc.

std_msgs/Header header
float64 data
