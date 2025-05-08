#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "controlper/mesa.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("restaurant_node");

	BT::BehaviorTreeFactory factory;
	BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("gotodoor_pfa_node"));
  factory.registerFromPlugin(loader.getOSName("countpeople_pfa_node"));
  factory.registerFromPlugin(loader.getOSName("checktable_pfa_node"));
  factory.registerFromPlugin(loader.getOSName("gototable_pfa_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("pfa_restaurante");
  std::string xml_file = pkgpath + "/behavior_tree_xml/restaurante.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  // Inicializamos mesas vacías
	controlper::Mesa mesa_big{6, false};
	controlper::Mesa mesa_small{2, false};
	blackboard->set("mesa_big", mesa_big);
	blackboard->set("mesa_small", mesa_small);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}