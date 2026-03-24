#ifndef EXOSKELETRON_SAFETY_MANAGER_SAFE_STOP_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_SAFE_STOP_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/BridgeModeClient.hpp"

namespace functional_safety
{
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

  // FIX BUG 2: shutdown() non manda più request_mode("nominal").
  // La responsabilità di impostare il bridge mode corretto è del
  // plugin ENTRANTE nel suo initialize(), non di quello USCENTE.
  // Il vecchio comportamento causava un transitorio pericoloso:
  // shutdown("nominal") → initialize("stop") con il bridge
  // brevemente in "nominal" senza protezione.
  void shutdown() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "SafeStopMode shutdown (bridge mode delegated to next plugin)");
    }
  }

  void set_safety_params(double) override {}
};
}
#endif