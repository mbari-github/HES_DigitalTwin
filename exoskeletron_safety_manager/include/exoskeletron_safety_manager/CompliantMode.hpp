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
  void shutdown() override { request_mode("nominal"); }
  void set_safety_params(double) override {}
};
}
#endif