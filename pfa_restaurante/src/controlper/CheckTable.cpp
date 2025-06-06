#include <string>
#include <iostream>
#include "controlper/mesa.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace controlper

{
  void hablarFestival(const std::string& texto)
  {
    std::string comando = "echo \"(voice_el_diphone)(SayText \\\"" + texto + "\\\")\" | festival --pipe";
    system(comando.c_str());
  }

class CheckTable : public BT::SyncActionNode
{
public:
  CheckTable(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("personas") };
  }

  BT::NodeStatus tick() override
  {
    std::cout << "CheckTable ejecutandose" << std::endl;

    int personas = 0;
    if (!getInput("personas", personas)) {
      std::cerr << "No se pudo obtener el número de personas desde la blackboard." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    auto bb = config().blackboard;
    Mesa big, small;

    if (!bb->get("mesa_big", big) || !bb->get("mesa_small", small)) {
      std::cerr << "No se pudieron obtener las mesas desde la blackboard." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (personas > 6) {
        hablarFestival("Ahora mismo no se encuentra una mesa disponible, lo sentimos");
      std::cout << "Mesa no disponible para " << personas << " personas." << std::endl;
      return BT::NodeStatus::FAILURE;
    } 
    else if (personas >= 4) {
      if (big.llena) {
        hablarFestival("Ahora mismo no se encuentra una mesa disponible, lo sentimos");
        std::cout << "No hay mesas disponibles (mesa BIG ya ocupada)." << std::endl;
        return BT::NodeStatus::FAILURE;
      }
      hablarFestival("Asignando mesa");
      std::cout << "Asignando a mesa BIG..." << std::endl;
      big.llena = true;
      bb->set("mesa_big", big);
      bb->set("destino", std::string("BIG"));
      std::cout << "Estado actualizado: " << big << std::endl;
    } 
    else {
      if (small.llena) {
        hablarFestival("Ahora mismo no se encuentra una mesa disponible, lo sentimos");
        std::cout << "No hay mesas disponibles (mesa SMALL ya ocupada)." << std::endl;
        return BT::NodeStatus::FAILURE;
      }
      hablarFestival("Asignando mesa");
      std::cout << "Asignando a mesa SMALL..." << std::endl;
      small.llena = true;
      bb->set("mesa_small", small);
      bb->set("destino", std::string("SMALL"));
      std::cout << "Estado actualizado: " << small << std::endl;
    }

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace controlper

// Registro del nodo en el BT factory
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::CheckTable>("CheckTable");
}
