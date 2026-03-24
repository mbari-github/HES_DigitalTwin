#ifndef EXOSKELETRON_SAFETY_MANAGER_TORQUE_LIMIT_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_TORQUE_LIMIT_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/BridgeModeClient.hpp"

namespace functional_safety
{
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

  // FIX BUG 2: shutdown() non manda più request_mode("nominal").
  void shutdown() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "TorqueLimitMode shutdown (bridge mode delegated to next plugin)");
    }
  }

  void set_safety_params(double) override {}
};
}
#endif