#include <string>
#include <iostream>
#include <limits>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace controlper
{

// Estructura de Mesa
struct Mesa
{
  int tamaño;
  bool estado; //false: mesa libre, true: mesa ocupada
};

//Sobrecarga de operador << para poder imprimir una mesa.
inline std::ostream& operator<<(std::ostream& os, const Mesa& mesa)
{
  os << "Mesa(tamaño=" << mesa.tamaño << ", estado=" << (mesa.estado ? "ocupada" : "libre") << ")";
  return os;
}

// Nodo que ejecuta la configuración inicial del BT
//primero pregunta cuántas personas hay 
//luego crea dos mesas BIG y SMALL y las guarda en la blackboard
class SetupInicial : public BT::SyncActionNode
{
public:
  SetupInicial(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()//se declara el puerto de salida de personas
  {
    return { BT::OutputPort<int>("personas") };  // Solo el número de personas va como puerto
  }

  BT::NodeStatus tick() override
  {
    if (!contarPersonas()) {
      return BT::NodeStatus::FAILURE;
    }

    crearMesas();

    return BT::NodeStatus::SUCCESS;
  }

private:
  //Función para pedir número de personas
  bool contarPersonas()
  {
    int personas = 0;
    std::cout << "¿Mesa para cuántos? ";
    std::cin >> personas;

    if (std::cin.fail() || personas <= 0) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cerr << "Número inválido.\n";
      return false;
    }

    if (!setOutput("personas", personas)) {
      std::cerr << "No se pudo guardar el número de personas en la blackboard.\n";
      return false;
    }

    std::cout << "Buscando mesa para " << personas << std::endl;
    return true;
  }

  //Función para crear mesas y guardarlas directamente en la blackboard
  void crearMesas()
  {
    Mesa big{6, false};   // BIG: tamaño 6
    Mesa small{4, false}; // SMALL: tamaño 4

    auto bb = config().blackboard;
    bb->set("mesa_big", big);
    bb->set("mesa_small", small);

    std::cout << "Mesas creadas y guardadas en la blackboard:\n";
    std::cout << " - mesa_big: " << big << "\n";
    std::cout << " - mesa_small: " << small << "\n";
  }
};

}  // namespace controlper

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::SetupInicial>("SetupInicial");
}
