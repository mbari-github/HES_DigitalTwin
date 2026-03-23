#ifndef EXOSKELETRON_SAFETY_MANAGER_BRIDGE_MODE_CLIENT_HPP
#define EXOSKELETRON_SAFETY_MANAGER_BRIDGE_MODE_CLIENT_HPP

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "exoskeletron_safety_msgs/srv/set_mode.hpp"

namespace functional_safety
{

// Utility base: chiama /bridge/set_mode con la stringa di modo richiesta.
// I plugin la ereditano e specificano solo quale modo richiedere.
class BridgeModeClient
{
protected:
  void init_bridge_client(const rclcpp::Node::SharedPtr & node)
  {
    node_ = node;
    client_ = node_->create_client<exoskeletron_safety_msgs::srv::SetMode>("/bridge/set_mode");
  }

  void request_mode(const std::string & mode)
  {
    if (!node_ || !client_) return;

    if (!client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_WARN(node_->get_logger(),
        "BridgeModeClient: /bridge/set_mode not available (requested mode='%s')", mode.c_str());
      return;
    }

    auto req = std::make_shared<exoskeletron_safety_msgs::srv::SetMode::Request>();
    req->mode = mode;
    auto future = client_->async_send_request(req);
    (void)future;

    RCLCPP_INFO(node_->get_logger(),
      "BridgeModeClient: requested mode='%s'", mode.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<exoskeletron_safety_msgs::srv::SetMode>::SharedPtr client_;
};

}  // namespace functional_safety
#endif