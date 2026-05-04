#ifndef EXOSKELETRON_SAFETY_MANAGER_SAFETY_TOOLS_HPP
#define EXOSKELETRON_SAFETY_MANAGER_SAFETY_TOOLS_HPP

#include <rclcpp/rclcpp.hpp>

namespace functional_safety
{

/**
 * SafetyTools — abstract base interface for pluginlib safety mode plugins.
 *
 * Each safety mode (FaultMonitor, CompliantMode, TorqueLimitMode, SafeStop)
 * is implemented as a plugin that inherits from this class. The StateMachine
 * loads and unloads plugins dynamically as the system transitions between states.
 *
 * Plugin lifecycle (managed by StateMachine::change_state):
 *   1. stop()       — called on the outgoing plugin before it is destroyed
 *   2. reset()      — the outgoing plugin's shared_ptr is released
 *   3. initialize() — called on the newly created incoming plugin
 *
 * Design invariant: a plugin must NOT set the bridge mode in shutdown().
 * Setting the bridge mode is the responsibility of the INCOMING plugin in
 * its initialize() call. Doing it in shutdown() creates a dangerous transient
 * where the bridge is briefly set to the wrong mode while the new plugin has
 * not yet started.
 */
class SafetyTools
{
public:
  virtual ~SafetyTools() {}

  /** Called once when the plugin is loaded. Set up subscribers, clients, timers here. */
  virtual void initialize(const rclcpp::Node::SharedPtr & node) = 0;

  /** Called before the plugin is unloaded. Release all ROS resources here. */
  virtual void stop() = 0;

  /** Pause the plugin's activity (optional — not all plugins need this). */
  virtual void pause() = 0;

  /** Resume after a pause. */
  virtual void resume() = 0;

  /** Release subscribers, clients, timers, etc. Called during destruction. */
  virtual void shutdown() = 0;

  /** Generic safety parameter setter (e.g. override the safe-stop torque threshold). */
  virtual void set_safety_params(double param) = 0;

protected:
  SafetyTools() {}
};

}  // namespace functional_safety

#endif
