//! LifecycleStateManager

#ifndef LIFECYCLE_STATE_MANAGER
#define LIFECYCLE_STATE_MANAGER

#include <cstdio>
#include <memory>
#include <queue>

#include "nodetemplate/lifecycle_node.hpp"
#include "nodetemplate/state_machine/lifecycle_state.hpp"

class LifecycleStateManager
{
public:
    // Singleton Methoden
    LifecycleStateManager(std::unique_ptr<LifecycleState>, rclcpp::Node*);
    static std::shared_ptr<LifecycleStateManager> GetStateManager();
    static std::shared_ptr<LifecycleStateManager> GetStateManager(std::unique_ptr<LifecycleState>, rclcpp::Node*);

    // Methode um den aktuellen State zu wechseln
    bool TransitionTo(std::unique_ptr<LifecycleState>);

    // Ruft Update-Methode des States auf
    void Update();    

    // Gibt Pointer auf die aktuelle Node zurück
    rclcpp::Node* GetNode();

    ~LifecycleStateManager();
private:
    LifecycleStateManager(const LifecycleStateManager&);
    LifecycleStateManager& operator=(const LifecycleStateManager&);

    // Pointer auf wichtige Objekte
    static std::shared_ptr<LifecycleStateManager> current_manager_;
    std::unique_ptr<LifecycleState> current_state_;
    rclcpp::Node* current_node_;

    // Eingereihter Übergang
    std::unique_ptr<LifecycleState> transition_;

    // Bearbeite eingereihten Übergang
    bool HandleTransitionQueue();
};

#endif