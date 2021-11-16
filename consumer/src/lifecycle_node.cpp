#include "nodetemplate/lifecycle_node.hpp"

#include "nodetemplate/state_machine/states/unconfigured_state.hpp"

// Initialisiere die Lifecyclenode mit dem Nodenamen und der UUID als Zusatz,
// um Dopplungen von Nodes zu vermeiden
LifecycleNode::LifecycleNode()
: Node(
    MetadataManager::GetMetadataManager()->GetFullNodeName()
),
node_start_time(
    std::chrono::steady_clock::now()
)
{
    RCLCPP_INFO(this->get_logger(), "Lifecycle node started");
    RCLCPP_INFO(this->get_logger(), "Node ID is '%s'", MetadataManager::GetMetadataManager()->GetUUID().str().c_str());

    RCLCPP_INFO(this->get_logger(), "Initializing lifecycle state manager with UnconfiguredState");

    // Initialisiere den Lifecycle-State-Manager
    std::unique_ptr<LifecycleState> initial_state = std::make_unique<UnconfiguredState>();
    LifecycleStateManager::GetStateManager(std::move(initial_state), this);

    // Initialisiere den Lifecycle-Update-Timer
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        []() { LifecycleStateManager::GetStateManager()->Update(); }
    );

    return;
}