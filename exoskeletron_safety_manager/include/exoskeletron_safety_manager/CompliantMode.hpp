#ifndef EXOSKELETRON_SAFETY_MANAGER_COMPLIANT_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_COMPLIANT_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/BridgeModeClient.hpp"

namespace functional_safety
{
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

  // FIX BUG 2: shutdown() non manda più request_mode("nominal").
  void shutdown() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "CompliantMode shutdown (bridge mode delegated to next plugin)");
    }
  }

  void set_safety_params(double) override {}
};
}
#endif