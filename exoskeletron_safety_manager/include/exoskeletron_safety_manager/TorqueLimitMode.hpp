#ifndef EXOSKELETRON_SAFETY_MANAGER_TORQUE_LIMIT_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_TORQUE_LIMIT_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/BridgeModeClient.hpp"

namespace functional_safety
{

/**
 * TorqueLimitMode — safety plugin that sets the bridge to 'torque_limit' mode.
 *
 * In torque_limit mode the bridge applies a hard cap on the commanded torque
 * (configured via bridge parameter override_tau_limit). This is used when a
 * moderate fault has been detected that requires a stricter torque bound than
 * compliant mode but does not yet justify a full stop.
 *
 * Bridge mode lifecycle:
 * - initialize(): requests bridge → 'torque_limit'
 * - stop() / resume(): re-asserts 'torque_limit' in case the bridge drifted
 * - shutdown(): does NOT send any mode request.
 *               The incoming plugin sets the correct bridge mode in its initialize().
 */
class TorqueLimitMode : public SafetyTools, protected BridgeModeClient
{
public:
  TorqueLimitMode() = default;
  ~TorqueLimitMode() override = default;

  void initialize(const rclcpp::Node::SharedPtr & node) override
  {
    init_bridge_client(node);
    RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode initialized");
    request_mode("torque_limit");
  }
  void stop()     override { request_mode("torque_limit"); }
  void pause()    override {}
  void resume()   override { request_mode("torque_limit"); }

  void shutdown() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode shutdown (bridge mode delegated to next plugin)");
    }
  }

  void set_safety_params(double) override {}
};
}  // namespace functional_safety
#endif
