#include <string>
#include <iostream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "nodetemplate/lifecycle_node.hpp"

#include "nodetemplate/metadata/metadata_manager.hpp"

#ifndef NODE_NAME
#define NODE_NAME "D455camConsumer"
#endif

// Einstiegsmethode der Node
int main(int argc, char * argv[])
{
  std::string node_name = NODE_NAME;  
  if (std::getenv("NT_NODE_NAME")) {
    node_name = std::string(std::getenv("NT_NODE_NAME"));
  }

  // Initialisiere ROS2
  rclcpp::init(argc, argv);

  // Initialisiere Metadaten-Manager
  MetadataManager::GetMetadataManager(node_name);

  // Initialisiere eine neue Lifecycle-Node
  std::shared_ptr<LifecycleNode> node = std::shared_ptr<LifecycleNode>(new LifecycleNode());

  // Starte die Node
  rclcpp::spin(node);

  // Beende ROS2
  rclcpp::shutdown();
  return 0;
}