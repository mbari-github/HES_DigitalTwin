#ifndef EXOSKELETRON_SAFETY_MANAGER_SAFE_STOP_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_SAFE_STOP_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace functional_safety
{

class SafeStopMode : public SafetyTools
{
public:
  SafeStopMode() = default;
  ~SafeStopMode() override = default;

  void initialize(const rclcpp::Node::SharedPtr & node) override;
  void stop() override;
  void pause() override;
  void resume() override;
  void shutdown() override;
  void set_safety_params(double param) override;

private:
  void call_stop_service(bool enable);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_client_;
  double param_{0.0};
};

} 

#endif 