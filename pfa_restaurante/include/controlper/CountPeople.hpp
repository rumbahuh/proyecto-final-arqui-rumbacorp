
#ifndef CONTROLPER__COUNTPEOPLE_HPP_
#define CONTROLPER__COUNTPEOPLE_HPP_

#include <string>
#include <iostream>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace controlper
{

// Nodo que ejecuta la configuración inicial del BT
// Primero pregunta cuántas personas hay, luego crea dos mesas BIG y SMALL
// y las guarda en la blackboard
class CountPeople : public BT::SyncActionNode
{
public:
  CountPeople(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();  // Solo el número de personas va como puerto

  BT::NodeStatus tick() override;

private:

  bool contarPersonas();

  // Función para crear mesas y guardarlas directamente en la blackboard
  void crearMesas();
};

}  // namespace controlper

#endif  // CONTROLPER__COUNTPEOPLE_HPP_
