#ifndef EXOSKELETRON_SAFETY_MANAGER_COMPLIANT_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_COMPLIANT_MODE_HPP

#include "exoskeleton_safety_manager/SafetyTools.hpp"
#include "exoskeleton_safety_manager/BridgeModeClient.hpp"

namespace functional_safety
{

/**
 * CompliantMode — safety plugin that sets the bridge to 'compliant' mode.
 *
 * In compliant mode the bridge applies reduced torque, velocity, and
 * acceleration limits (configured in the bridge parameters). This is
 * used when a soft fault has been detected but the system can continue
 * operating under tighter constraints.
 *
 * Bridge mode lifecycle:
 * - initialize(): requests bridge → 'compliant'
 * - stop() / resume(): re-asserts 'compliant' in case the bridge drifted
 * - shutdown(): does NOT send any mode request.
 *               Setting the bridge mode at shutdown would create a dangerous
 *               transient: the bridge could briefly be in the wrong mode before
 *               the incoming plugin sets the correct one. Mode responsibility
 *               always belongs to the INCOMING plugin's initialize().
 */
class CompliantMode : public SafetyTools, protected BridgeModeClient
{
public:
  CompliantMode() = default;
  ~CompliantMode() override = default;

  void initialize(const rclcpp::Node::SharedPtr & node) override
  {
    init_bridge_client(node);
    RCLCPP_INFO(node_->get_logger(), "CompliantMode initialized");
    request_mode("compliant");
  }
  void stop()     override { request_mode("compliant"); }
  void pause()    override {}
  void resume()   override { request_mode("compliant"); }

  void shutdown() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "CompliantMode shutdown (bridge mode delegated to next plugin)");
    }
  }

  void set_safety_params(double) override {}
};
}  // namespace functional_safety
#endif
