#ifndef EXOSKELETRON_SAFETY_MANAGER_BRIDGE_MODE_CLIENT_HPP
#define EXOSKELETRON_SAFETY_MANAGER_BRIDGE_MODE_CLIENT_HPP

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "exoskeletron_safety_msgs/srv/set_mode.hpp"

namespace functional_safety
{

/**
 * BridgeModeClient — mixin that gives a plugin the ability to request a
 * bridge mode change via the /bridge/set_mode service.
 *
 * Usage: inherit from this class (protected) and call init_bridge_client()
 * in initialize(), then call request_mode("nominal"|"compliant"|...) as needed.
 *
 * Design notes:
 * - request_mode() is fully asynchronous (async_send_request + callback).
 *   It does NOT block the calling thread and works with a single-threaded executor.
 * - If the service is not available within 500 ms, a warning is logged and the
 *   request is silently dropped. Escalation to SAFE_STOP in case of a dead bridge
 *   is the responsibility of StateMachine::check_bridge_liveness(), NOT of this
 *   class — doing it here would create an infinite loop when the bridge is dead.
 * - last_confirmed_mode_ tracks the last mode that was positively acknowledged
 *   by the bridge, which can be used for diagnostics.
 */
class BridgeModeClient
{
protected:
  void init_bridge_client(const rclcpp::Node::SharedPtr & node)
  {
    node_ = node;
    client_ = node_->create_client<exoskeletron_safety_msgs::srv::SetMode>(
      "/bridge/set_mode");
  }

  void request_mode(const std::string & mode)
  {
    if (!node_ || !client_) return;

    if (!client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_WARN(node_->get_logger(),
        "BridgeModeClient: /bridge/set_mode not available "
        "(requested mode='%s')", mode.c_str());
      // Do NOT escalate here — the state machine will detect the dead bridge
      // via check_bridge_liveness() and handle the SAFE_STOP transition.
      return;
    }

    auto req = std::make_shared<exoskeletron_safety_msgs::srv::SetMode::Request>();
    req->mode = mode;

    auto response_cb = [this, mode](
      rclcpp::Client<exoskeletron_safety_msgs::srv::SetMode>::SharedFuture future)
    {
      try {
        auto response = future.get();
        if (response->success) {
          last_confirmed_mode_ = mode;
          RCLCPP_INFO(node_->get_logger(),
            "BridgeModeClient: mode='%s' CONFIRMED", mode.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(),
            "BridgeModeClient: mode='%s' REJECTED | msg='%s'",
            mode.c_str(), response->message.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(),
          "BridgeModeClient: mode='%s' exception: %s",
          mode.c_str(), e.what());
      }
    };

    client_->async_send_request(req, response_cb);

    RCLCPP_INFO(node_->get_logger(),
      "BridgeModeClient: requested mode='%s'", mode.c_str());
  }

  std::string get_last_confirmed_mode() const { return last_confirmed_mode_; }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<exoskeletron_safety_msgs::srv::SetMode>::SharedPtr client_;

private:
  std::string last_confirmed_mode_{"unknown"};
};

}  // namespace functional_safety
#endif
