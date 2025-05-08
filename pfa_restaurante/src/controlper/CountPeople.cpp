#include "controlper/CountPeople.hpp"
#include <iostream>
#include <limits>
#include "controlper/mesa.hpp"

namespace controlper
{
  // Usa la voz del festival en español (latino)
  void hablarFestival(const std::string& texto)
  {
    std::string comando = "echo \"(voice_el_diphone)(SayText \\\"" + texto + "\\\")\" | festival --pipe";
    system(comando.c_str());
  }

// Nodo que ejecuta la configuración inicial del BT
// primero pregunta cuántas personas hay 
// luego crea dos mesas BIG y SMALL y las guarda en la blackboard
CountPeople::CountPeople(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

// Define los puertos proporcionados por este nodo
BT::PortsList CountPeople::providedPorts() // puerto de salida de personas
{
  return { BT::OutputPort<int>("personas") };
}

BT::NodeStatus CountPeople::tick()
{
  std::cout << "CountPeople ejecutandose" << std::endl;

  if (!contarPersonas()) {
    return BT::NodeStatus::FAILURE;
  }

  // Si contar personas tiene éxito, creamos las mesas
  crearMesas();
  
  return BT::NodeStatus::SUCCESS;
}

bool CountPeople::contarPersonas()
{
  int personas = 0;
  hablarFestival("Por favor, introduzca para cuántos será la mesa");
  std::cout << "¿Mesa para cuántos?\n";
  std::cin >> personas;

  // Validamos que el número ingresado sea válido
  if (std::cin.fail() || personas <= 0) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    hablarFestival("Número inválido");
    std::cerr << "Número inválido.\n";
    return false; // Si es inválido, devolvemos false
  }

  // Guardamos el número de personas en la blackboard
  if (!setOutput("personas", personas)) {
    std::cerr << "No se pudo guardar el número de personas en la blackboard.\n";
    return false; 
  }

  hablarFestival("Buscando su mesa, por favor espere");
  std::cout << "Buscando mesa para " << personas << std::endl;
  return true;
}

void CountPeople::crearMesas()
{
  Mesa big{6, false};   // BIG: tamaño 6
  Mesa small{4, false}; // SMALL: tamaño 4

  // Guardamos las mesas en la blackboard
  auto bb = config().blackboard;
  bb->set("mesa_big", big);
  bb->set("mesa_small", small);

  // Imprimimos el estado de las mesas
  std::cout << "Mesas creadas y guardadas en la blackboard:";
  std::cout << " - mesa_big: " << big << "\n";
  std::cout << " - mesa_small: " << small << "\n";
}

}  // namespace controlper

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::CountPeople>("CountPeople");
}

/
{
  Mesa big{6, false};
  Mesa small{4, false};

  auto bb = config().blackboard;
  bb->set("mesa_big", big);
  bb->set("mesa_small", small);

  std::cout << "Mesas creadas y guardadas en la blackboard:\n";
  std::cout << " - mesa_big: " << big << "\n";
  std::cout << " - mesa_small: " << small << "\n";
}
