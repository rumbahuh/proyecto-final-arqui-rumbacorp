#ifndef CONTROL_CHECKTABLE_HPP
#define CONTROL_CHECKTABLE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <string>
#include <iostream>

namespace controlper
{

// Estructura Mesa compatible con la definición del sistema
struct Mesa
{
  int tamaño;
  bool estado;  // true = ocupada, false = libre
};

// Para imprimir mesas en consola
inline std::ostream& operator<<(std::ostream& os, const Mesa& mesa)
{
  os << "Mesa(tamaño=" << mesa.tamaño << ", estado=" << (mesa.estado ? "ocupada" : "libre") << ")";
  return os;
}

// Nodo de acción que asigna una mesa en base al número de personas
class CheckTable : public BT::SyncActionNode
{
public:
  CheckTable(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace controlper

#endif // CONTROL_CheckTable_HPP










