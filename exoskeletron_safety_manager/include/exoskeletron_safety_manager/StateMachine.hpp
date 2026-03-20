#ifndef EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include "exoskeletron_safety_manager/safety_states.hpp"

// questi include li aggiorneremo quando creerai i .srv reali
#include "std_srvs/srv/set_bool.hpp"

namespace functional_safety
{

class StateMachine : public rclcpp::Node
{
public:
  StateMachine();
  void initialize();

private:
  void change_state(const std::string & state);

  // callback servizi
  void safe_stop_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void compliant_mode_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void torque_limit_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void sensor_degraded_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

private:
  functional_safety::SafetyState current_state_;

  pluginlib::ClassLoader<functional_safety::SafetyTools> state_loader_;
  std::shared_ptr<functional_safety::SafetyTools> obj_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr safe_stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr compliant_mode_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_limit_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sensor_degraded_service_;
};

}

#endif  // EXOSKELETRON_SAFETY_MANAGER_STATE_MACHINE_HPP