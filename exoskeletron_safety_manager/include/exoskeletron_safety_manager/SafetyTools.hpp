#ifndef EXOSKELETRON_SAFETY_MANAGER_SAFETY_TOOLS_HPP
#define EXOSKELETRON_SAFETY_MANAGER_SAFETY_TOOLS_HPP

#include <rclcpp/rclcpp.hpp>

namespace functional_safety
{
class SafetyTools
{
public:
  virtual ~SafetyTools() {}

  // inizializzazione del plugin sul nodo state_machine
  virtual void initialize(const rclcpp::Node::SharedPtr & node) = 0;

  // arresto / pausa / ripresa della modalità
  virtual void stop() = 0;
  virtual void pause() = 0;
  virtual void resume() = 0;

  // rilascio subscriber, client, timer, ecc.
  virtual void shutdown() = 0;

  // parametro generico di sicurezza per la modalità attiva
  virtual void set_safety_params(double param) = 0;

protected:
  SafetyTools() {}
};

} 

#endif  