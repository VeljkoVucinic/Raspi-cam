//consumer
#include "nodetemplate/state_machine/states/active_state.hpp"

ActiveState::ActiveState() {}

bool ActiveState::OnBegin()
{
    // Setting current node to active
    NodeNetworkManager::GetNodeNetworkManager()->SetActive(true);
    return true;
}

void ActiveState::OnError()
{

}

void ActiveState::OnUpdate()
{
    // Sende Heartbeat
    NodeNetworkManager::GetNodeNetworkManager()->Heartbeat();


    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();
    std::vector<Dependency>* deps = mdm->GetDependencies();

    // Sende eine Suche, nach allen noch nicht erfüllten Abhängigkeiten
    for (std::vector<Dependency>::iterator dep = deps->begin(); dep != deps->end(); ++dep) {
        // Wenn die Abhängigkeit erfüllt ist
        if (dep->Satisfied != nullptr) {
            // Prüfen, ob eine Heartbeat empfangen wurde
            double DepBondInterval = 1000.0 / dep->BondRate;
          
            std::chrono::time_point<std::chrono::steady_clock> LastBondIntervalMinimum = std::chrono::steady_clock::now() - std::chrono::milliseconds(int(ceil(DepBondInterval)));
            std::chrono::time_point<std::chrono::steady_clock> KnockOutTimestamp = std::chrono::steady_clock::now() - std::chrono::milliseconds(int(ceil(DepBondInterval)) * dep->MaxBondMisses);

            if (dep->Satisfied->LastBond < LastBondIntervalMinimum) {
                //RCLCPP_INFO(rclcpp::get_logger("ActiveState::OnUpdate"), "Dependency '%s' missed heartbeat", dep->InternalName.c_str());
                if (dep->Satisfied->LastBond < KnockOutTimestamp) {
                    RCLCPP_INFO(rclcpp::get_logger("ActiveState::OnUpdate"), "Dependency '%s' gone!", dep->InternalName.c_str());

                    for (std::vector<DependencyCandidate*>::iterator cand_iter = dep->Candidates.begin(); cand_iter != dep->Candidates.end(); ++cand_iter) {
                        if (*cand_iter == dep->Satisfied) {
                            dep->Candidates.erase(cand_iter);
                        }
                    }
                    delete dep->Satisfied;
                    dep->Satisfied = nullptr;

                    sort(dep->Candidates.begin(), dep->Candidates.end(), [](const DependencyCandidate* lhs, const DependencyCandidate* rhs) -> bool {
                            return lhs->Rating < rhs->Rating;
                        });

                    for (std::vector<DependencyCandidate*>::iterator cand_iter = dep->Candidates.begin(); cand_iter != dep->Candidates.end(); ++cand_iter) {
                        if ((*cand_iter)->LastBond < LastBondIntervalMinimum) {
                            dep->Satisfied = *cand_iter;
                            NodeNetworkManager::GetNodeNetworkManager()->OnNewDependency(*dep);
                            RCLCPP_INFO(rclcpp::get_logger("ActiveState::OnUpdate"), "Found instant replacement for dependency '%s'.", dep->InternalName.c_str());
                            break;
                        }
                    }

                    if (!dep->Soft && dep->Satisfied == nullptr) {
                        // Wenn eine "harte" Abhängigkeit verloren wurde, kann diese Node nicht mehr aktiv sein!
                        RCLCPP_INFO(rclcpp::get_logger("ActiveState::OnUpdate"), "Hard dependency lost!");
                        LifecycleStateManager::GetStateManager()->TransitionTo(std::make_unique<InactiveState>());
                    }
                    NodeNetworkManager::GetNodeNetworkManager()->RefreshPnP();
                }
                continue;
            }
        }

        //!@ Aktionen die von spezifischen Abhängigkeiten abhängen hier hinzufügen
    }
}


bool ActiveState::OnTransition(std::string new_state)
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
std::string ActiveState::GetName()
{
    return "active_state";
}
