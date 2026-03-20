#ifndef EXOSKELETRON_SAFETY_MANAGER_SENSOR_DEGRADED_MODE_HPP
#define EXOSKELETRON_SAFETY_MANAGER_SENSOR_DEGRADED_MODE_HPP

#include "exoskeletron_safety_manager/SafetyTools.hpp"
#include <rclcpp/rclcpp.hpp>

namespace functional_safety
{

class SensorDegradedMode : public SafetyTools
{
public:
  void initialize(const rclcpp::Node::SharedPtr & node) override;
  void stop() override;
  void pause() override;
  void resume() override;
  void shutdown() override;
  void set_safety_params(double param) override;

private:
  rclcpp::Node::SharedPtr node_;
};

}

#endif