#include <string>
#include <iostream>
#include <limits>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace controlper
{

// Estructura de mesa
struct Mesa
{
  int tamaño;
  bool estado;
};

// Para permitir que se imprima (debug/log)
inline std::ostream& operator<<(std::ostream& os, const Mesa& mesa)
{
  os << "Mesa(tamaño=" << mesa.tamaño << ", estado=" << (mesa.estado ? "ocupada" : "libre") << ")";
  return os;
}

// Nodo que pide el número de personas y crea mesas en la blackboard
class SetupMesasYPersonas : public BT::SyncActionNode
{
public:
  SetupMesasYPersonas(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    // Solo se declara como output el número de personas, para usar setOutput
    return { BT::OutputPort<int>("personas_out") };
  }

  BT::NodeStatus tick() override
  {
    int personas = 0;
    std::cout << "¿Cuántas personas hay? ";
    std::cin >> personas;

    if (std::cin.fail() || personas <= 0) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cerr << "Número inválido de personas.\n";
      return BT::NodeStatus::FAILURE;
    }

    // ✅ Publicar el número de personas mediante setOutput
    if (!setOutput("personas_out", personas)) {
      std::cerr << "No se pudo publicar el número de personas.\n";
      return BT::NodeStatus::FAILURE;
    }

    std::cout << "Número de personas registrado: " << personas << std::endl;

    // ✅ Crear mesas y guardarlas directamente en la blackboard
    Mesa big{6, false};   // tamaño 6, libre
    Mesa small{4, false}; // tamaño 4, libre

    // Blackboard directa (sin pasar por los puertos)
    auto bb = config().blackboard;
    bb->set("mesa_big", big);
    bb->set("mesa_small", small);

    std::cout << "Mesas BIG y SMALL guardadas en la blackboard." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace controlper

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::SetupMesasYPersonas>("SetupMesasYPersonas");
}
