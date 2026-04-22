# Float array with timestamp — replaces std_msgs/Float64MultiArray
# for all topics that require time synchronization.
#
# Usage: ff_terms, debug vectors, trajectory_ref, etc.

std_msgs/Header header
float64[] data
