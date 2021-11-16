//consumer
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
    //! Dependency for DEPTH sensor
    Dependency DepthDependency = Dependency();
    DepthDependency.InternalName = "Depth_dependency";
    DepthDependency.Cap = { "Depth", 0 };
    DepthDependency.Soft = false;
    DepthDependency.Bond = HEARTBEAT;
    DepthDependency.BondRate = 1.0;
    DepthDependency.MaxBondMisses = 5;
    DepthDependency.MetaData = {};
    DepthDependency.Satisfied = nullptr;
    DepthDependency.CandidateRatingFunction = [](DependencyCandidate* candidate) -> double {
        return candidate->Error;
    };
    md_mgr->AddDependency(DepthDependency);

    //Dependency for COLOR sensor
    Dependency ColorDependency = Dependency();
    ColorDependency.InternalName = "Color_dependency";
    ColorDependency.Cap = { "Color", 0 };
    ColorDependency.Soft = false;
    ColorDependency.Bond = HEARTBEAT;
    ColorDependency.BondRate = 1.0;
    ColorDependency.MaxBondMisses = 5;
    ColorDependency.MetaData = {};
    ColorDependency.Satisfied = nullptr;
    ColorDependency.CandidateRatingFunction = [](DependencyCandidate* candidate) -> double {
        return candidate->Error;
    };
    md_mgr->AddDependency(ColorDependency);

    //Dependency for gyro sensor
    Dependency GyroDependency = Dependency();
    GyroDependency.InternalName = "Gyro_dependency";
    GyroDependency.Cap = { "Gyro", 0 };
    GyroDependency.Soft = false;
    GyroDependency.Bond = HEARTBEAT;
    GyroDependency.BondRate = 1.0;
    GyroDependency.MaxBondMisses = 5;
    GyroDependency.MetaData = {};
    GyroDependency.Satisfied = nullptr;
    GyroDependency.CandidateRatingFunction = [](DependencyCandidate* candidate) -> double {
        return candidate->Error;
    };
    md_mgr->AddDependency(GyroDependency);

    //Dependency for accel sensor
    Dependency AccelDependency = Dependency();
    AccelDependency.InternalName = "Accel_dependency";
    AccelDependency.Cap = { "Accel", 0 };
    AccelDependency.Soft = false;
    AccelDependency.Bond = HEARTBEAT;
    AccelDependency.BondRate = 1.0;
    AccelDependency.MaxBondMisses = 5;
    AccelDependency.MetaData = {};
    AccelDependency.Satisfied = nullptr;
    AccelDependency.CandidateRatingFunction = [](DependencyCandidate* candidate) -> double {
        return candidate->Error;
    };
    md_mgr->AddDependency(AccelDependency);


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
    }
    else {
        return false;
    }
}

// Der Name dieses States
std::string UnconfiguredState::GetName()
{
    return "unconfigured_state";
}