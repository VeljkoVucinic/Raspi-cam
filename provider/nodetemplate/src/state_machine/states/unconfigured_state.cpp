//provider
//! Dieser State initialisiert Hard- und Software, sowie die ROS2-Endpoints (Publisher, Subscriber, Actions, Services, ...)

#include "nodetemplate/state_machine/states/unconfigured_state.hpp"

UnconfiguredState::UnconfiguredState() {}

bool UnconfiguredState::OnBegin()
{
    // NodeNetworkManager initialisieren und abrufen
    std::shared_ptr<NodeNetworkManager> nnm = NodeNetworkManager::GetNodeNetworkManager();

    // MetadataManager abrufen
    std::shared_ptr<MetadataManager> md_mgr = MetadataManager::GetMetadataManager();
    
    // Aktuellen Node auf inaktiv stellen
    nnm->SetActive(false);

    //!@ Hier kann Hardware usw. initialisiert werden
    
    Capability RaspiColorCapability = Capability();
    RaspiColorCapability.Tag = "Color";
    RaspiColorCapability.Error = 32767;
    md_mgr->SetCapability(RaspiColorCapability);
    

    // ROS2-Endpoints einrichten
    bool endpoints_initiated = nnm->InitiateEndpoints();
    md_mgr->SetConfigured(true);

    return endpoints_initiated;    
}

void UnconfiguredState::OnError()
{

}

void UnconfiguredState::OnUpdate()
{
    // Dieser State muss nur einmalig initialisieren. Dementsprechend kann direkt zum inactive state gewechselt werden.
    LifecycleStateManager::GetStateManager()->TransitionTo(std::make_unique<InactiveState>());
}


bool UnconfiguredState::OnTransition(std::string new_state)
{
    // Überprüfen, ob dieser State zu new_state übergehen kann
    auto it = ((this->allowed_transitions_).find(new_state));
    if (it != (this->allowed_transitions_).end()) {
        return (it->second)();
    } else {
        return false;
    }
}

// Der Name dieses States
std::string UnconfiguredState::GetName()
{
    return "unconfigured_state";
}