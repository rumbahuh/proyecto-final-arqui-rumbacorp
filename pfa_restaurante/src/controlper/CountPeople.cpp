#include "controlper/CountPeople.hpp"
#include <iostream>
#include <limits>
#include "controlper/mesa.hpp"

namespace controlper
{

// Nodo que ejecuta la configuración inicial del BT
// primero pregunta cuántas personas hay 
// luego crea dos mesas BIG y SMALL y las guarda en la blackboard
CountPeople::CountPeople(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

// Define los puertos proporcionados por este nodo
BT::PortsList CountPeople::providedPorts() // se declara el puerto de salida de personas
{
  return { BT::OutputPort<int>("personas") };  // Solo el número de personas va como puerto
}

// Método principal que se ejecuta cuando se llama al nodo
BT::NodeStatus CountPeople::tick()
{
  // Primero contamos las personas
  std::cout << "CountPeople ejecutandose" << std::endl;

  if (!contarPersonas()) {
    return BT::NodeStatus::FAILURE; // Si falla, devuelve FAILURE
  }

  // Si contar personas tiene éxito, creamos las mesas
  crearMesas();

  // Si todo salió bien, devolvemos SUCCESS
  return BT::NodeStatus::SUCCESS;
}

// Función para pedir número de personas
bool CountPeople::contarPersonas()
{
  int personas = 0;
  std::cout << "¿Mesa para cuántos?\n";
  std::cin >> personas;

  // Validamos que el número ingresado sea válido
  if (std::cin.fail() || personas <= 0) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cerr << "Número inválido.\n";
    return false; // Si es inválido, devolvemos false
  }

  // Guardamos el número de personas en la blackboard
  if (!setOutput("personas", personas)) {
    std::cerr << "No se pudo guardar el número de personas en la blackboard.\n";
    return false; // Si no se pudo guardar, devolvemos false
  }

  std::cout << "Buscando mesa para " << personas << std::endl;
  return true; // Si todo es válido, devolvemos true
}

// Función para crear mesas y guardarlas directamente en la blackboard
void CountPeople::crearMesas()
{
  // Creamos dos mesas: una grande y una pequeña
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

/*
#include "controlper/CountPeople.hpp"
#include "hri/dialog/speak.hpp"
#include "hri/dialog/listen.hpp"
#include <iostream>
#include <limits>
#include <sstream>

namespace controlper
{

using namespace dialog;

CountPeople::CountPeople(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList CountPeople::providedPorts()
{
  return { BT::OutputPort<int>("personas") };
}

BT::NodeStatus CountPeople::tick()
{
  if (!contarPersonas()) {
    return BT::NodeStatus::FAILURE;
  }

  crearMesas();
  return BT::NodeStatus::SUCCESS;
}

bool CountPeople::contarPersonas()
{
  // Crear instancia del nodo Speak
  Speak speak_node("speak_personas", "say", config());
    std::cerr << "Error al obtener el texto escuchado.\n";
    return false;
  }

  std::cout << "Texto escuchado: " << escuchado << std::endl;

  // Convertir a entero
  int personas = 0;
  std::stringstream ss(escuchado);
  ss >> personas;

  if (ss.fail() || personas <= 0) {
    std::cerr << "Número inválido escuchado.\n";
    return false;
  }

  if (!setOutput("personas", personas)) {
    std::cerr << "No se pudo guardar el número de personas en la blackboard.\n";
    return false;
  }

  std::cout << "Buscando mesa para " << personas << std::endl;
  return true;
}

void CountPeople::crearMesas()
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

}  // namespace controlper
*/
