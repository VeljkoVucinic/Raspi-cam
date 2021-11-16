//! Hauptklasse dieser Node
#pragma once

#include <chrono>
#include <memory>

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/node.hpp"

#include "nodetemplate/state_machine/lifecycle_state_manager.hpp"

#include "nodetemplate/metadata/metadata_manager.hpp"

class LifecycleNode : public rclcpp::Node
{
public:
    LifecycleNode();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::time_point<std::chrono::steady_clock> node_start_time;
};