
#ifndef CONTROLPER__COUNTPEOPLE_HPP_
#define CONTROLPER__COUNTPEOPLE_HPP_

#include <string>
#include <iostream>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace controlper
{

// Estructura de Mesa
struct Mesa
{
  int tamaño;
  bool llena; 
  
  friend std::ostream& operator<<(std::ostream& os, const Mesa& mesa) {
    os << "{capacidad: " << mesa.tamaño << ", llena: " << (mesa.llena ? "sí" : "no") << "}";
    return os;
  }
};

// Sobrecarga de operador << para poder imprimir una mesa
inline std::ostream& operator<<(std::ostream& os, const Mesa& mesa)
{
  os << "Mesa(tamaño=" << mesa.tamaño << ", estado=" << (mesa.estado ? "ocupada" : "libre") << ")";
  return os;
}

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
  // Función para pedir número de personas
  bool contarPersonas();

  // Función para crear mesas y guardarlas directamente en la blackboard
  void crearMesas();
};

}  // namespace controlper

#endif  // CONTROLPER__COUNTPEOPLE_HPP_
/*
#pragma once

#include "behaviortree_cpp_v3/action_node.h"

namespace controlper
{

class CountPeople : public BT::SyncActionNode
{
public:
  CountPeople(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool contarPersonas();
  void crearMesas();
};

}  // namespace controlper
*/
