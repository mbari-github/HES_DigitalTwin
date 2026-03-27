#ifndef EXOSKELETRON_SAFETY_MANAGER_BRIDGE_MODE_CLIENT_HPP
#define EXOSKELETRON_SAFETY_MANAGER_BRIDGE_MODE_CLIENT_HPP

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "exoskeletron_safety_msgs/srv/set_mode.hpp"

namespace functional_safety
{

// ════════════════════════════════════════════════════════════════════
// BridgeModeClient — callback asincrono di verifica
// ════════════════════════════════════════════════════════════════════
//
// Rispetto alla versione originale ((void)future):
//   - request_mode() registra un response_callback che logga se il
//     bridge ha accettato o rifiutato il cambio modo
//   - last_confirmed_mode_ traccia l'ultimo modo confermato
//   - Se il servizio non è disponibile, logga ERROR e ritorna
//     (l'escalation a SAFE_STOP è responsabilità della SM via
//     check_bridge_liveness, NON del BridgeModeClient — altrimenti
//     si crea un loop infinito quando il bridge è morto)
//
// Compatibilità: nessun spin_until_future_complete, nessun secondo
// client di servizio, funziona con il single-threaded executor.

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
      // NON escalare qui — la SM rileverà la morte del bridge
      // via check_bridge_liveness() e gestirà il SAFE_STOP.
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