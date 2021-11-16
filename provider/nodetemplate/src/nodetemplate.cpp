#include <string>
#include <iostream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "nodetemplate/lifecycle_node.hpp"

#include "nodetemplate/metadata/metadata_manager.hpp"

#ifndef NODE_NAME
#define NODE_NAME "D455camProvider" //changed from "..._gps"
#endif

// Einstiegsmethode der Node
int main(int argc, char * argv[])
{
  //if(std::getenv("GPS_DEV_PATH")) {
    // Initialisiere ROS2
    rclcpp::init(argc, argv);

    // Initialisiere Metadaten-Manager
    MetadataManager::GetMetadataManager(NODE_NAME);

    // Initialisiere eine neue Lifecycle-Node
    std::shared_ptr<LifecycleNode> node = std::shared_ptr<LifecycleNode>(new LifecycleNode());

    // Starte die Node
    rclcpp::spin(node);

    // Beende ROS2
    rclcpp::shutdown();
  //} else {
    //std::cout << "Environment variable 'GPS_DEV_PATH' not found!" << std::endl;
  //}
  return 0;
}