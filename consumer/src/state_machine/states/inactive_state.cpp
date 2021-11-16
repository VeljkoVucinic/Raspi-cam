//consumer
#include "nodetemplate/state_machine/states/inactive_state.hpp"


InactiveState::InactiveState() {}

bool InactiveState::OnBegin()
{
    // Die Node bleibt auch hier weiterhin inaktiv
    NodeNetworkManager::GetNodeNetworkManager()->SetActive(false);
    return true;
}

void InactiveState::OnError()
{

}

void InactiveState::OnUpdate()
{
    // Wenn alle "harten" Abhängigkeiten erfüllt sind, kann in den active state gewechselt werden.
    // Abhängigkeiten werden im NodeNetworkManager gesucht
    if (this->AreAllDependenciesSatisfied()) {
        LifecycleStateManager::GetStateManager()->TransitionTo(std::make_unique<ActiveState>());
    }
}

bool InactiveState::AreAllDependenciesSatisfied()
{
    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();

    // Überprüfen, ob alle "harten" Abhängigkeiten erfüllt sind
    for (std::vector<Dependency>::iterator it = mdm->GetDependencies()->begin(); it != mdm->GetDependencies()->end(); ++it) {
        if (it->Satisfied == nullptr && !it->Soft) {
            return false;
        }
    }
    return true;
}

bool InactiveState::OnTransition(std::string new_state)
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
std::string InactiveState::GetName()
{
    return "inactive_state";
}