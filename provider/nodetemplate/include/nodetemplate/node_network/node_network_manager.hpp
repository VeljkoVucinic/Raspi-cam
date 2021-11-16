//! Raspi provider

#ifndef NODE_NETWORK_MANAGER
#define NODE_NETWORK_MANAGER
//#ifndef PI_CAMERA
//#define PI_CAMERA

#include <sstream>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>
#include "nodetemplate/utilities/base64/base64.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//ovde
//#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "nodetemplate/utilities/json/json.hpp"
#include "nodetemplate/utilities/base64/base64.h"

#include "nodetemplate/metadata/metadata_manager.hpp"
#include "nodetemplate/state_machine/lifecycle_state_manager.hpp"

#include <memory>
#include <functional>
#include <cmath>
#include <chrono>
#include <thread>


class NodeNetworkManager {
public:
    NodeNetworkManager();
    static std::shared_ptr<NodeNetworkManager> GetNodeNetworkManager();
    ~NodeNetworkManager();

    bool InitiateEndpoints();
    
    bool IsActive();
    void SetActive(bool);

    void RefreshPnP();
    void AnnouncePnP();
    void Heartbeat();
    //ovde
    void SendFrame(nlohmann::json);

    void OnNewDependency(Dependency);

private:
    NodeNetworkManager& operator=(const NodeNetworkManager&);
    static std::shared_ptr<NodeNetworkManager> current_manager_;

    bool is_active_;

    //raspicam
    raspicam::RaspiCam Camera;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pnp_discovery_publisher_;
    void pnp_discovery_announce();
    void pnp_discovery_search();
    bool pnp_should_swap_dependency(std::vector<Dependency>::iterator);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
    rclcpp::TimerBase::SharedPtr heartbeat_publisher_timer_;
    void heartbeat_publisher_timer_function();

    //ovde
    std::thread* raspi_frame_thread;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raspi_publisher;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pnp_discovery_subscriber_;
    void pnp_discovery_subscriber_function(const std_msgs::msg::String::SharedPtr);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_subscriber_;
    void pnp_heartbeat_subscriber_function(const std_msgs::msg::String::SharedPtr);

    // Services
};

#endif
