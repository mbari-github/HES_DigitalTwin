#ifndef EXOSKELETRON_SAFETY_MANAGER_SAFE_STOP_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_SAFE_STOP_MODE_HPP

#include "exoskeleton_safety_manager/SafetyTools.hpp"
#include "exoskeleton_safety_manager/BridgeModeClient.hpp"

namespace functional_safety
{

/**
 * SafeStopMode — safety plugin that sets the bridge to 'stop' mode.
 *
 * In stop mode the bridge drives the torque to zero and holds the joint
 * at its current position. This state is latched: the system can only exit
 * it via an explicit /reset_safety_request service call.
 *
 * Bridge mode lifecycle:
 * - initialize(): requests bridge → 'stop'
 * - stop() / resume(): re-asserts 'stop' in case the bridge drifted
 * - shutdown(): does NOT send any mode request.
 *               The incoming plugin sets the bridge mode in its own initialize().
 *               Sending 'nominal' here would create a dangerous window where
 *               the bridge has no torque limit while the next plugin initializes.
 */
class SafeStopMode : public SafetyTools, protected BridgeModeClient
{
public:
  SafeStopMode() = default;
  ~SafeStopMode() override = default;

  void initialize(const rclcpp::Node::SharedPtr & node) override
  {
    init_bridge_client(node);
    RCLCPP_INFO(node_->get_logger(), "SafeStopMode initialized");
    request_mode("stop");
  }
  void stop()     override { request_mode("stop"); }
  void pause()    override {}
  void resume()   override { request_mode("stop"); }

  void shutdown() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "SafeStopMode shutdown (bridge mode delegated to next plugin)");
    }
  }

  void set_safety_params(double) override {}
};
}  // namespace functional_safety
#endif
