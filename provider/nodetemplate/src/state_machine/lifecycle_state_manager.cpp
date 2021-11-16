
#include "nodetemplate/state_machine/lifecycle_state_manager.hpp"

#include "rclcpp/rclcpp.hpp"

std::shared_ptr<LifecycleStateManager> LifecycleStateManager::current_manager_;

LifecycleStateManager::LifecycleStateManager(std::unique_ptr<LifecycleState> initial_state, rclcpp::Node* current_node)
{
    // Dieser Konstrutktor wird von der Methode LifecycleStateManager::GetStateManager aufgerufen, um ein Singleton zu erstellen.
    // Deswegen sollte diese Methode nicht von Hand aufgerufen werden.
    RCLCPP_INFO(rclcpp::get_logger("LifecycleStateManager"), "New LSM with initial state '%s'", initial_state->GetName().c_str());
    this->current_state_ = nullptr;
    this->current_node_ = current_node;
    this->transition_ = std::move(initial_state);
}

LifecycleStateManager::~LifecycleStateManager() {}

std::shared_ptr<LifecycleStateManager> LifecycleStateManager::GetStateManager()
{
    if (!LifecycleStateManager::current_manager_) {
        // Gibt einen Nullpointer zurück, wenn kein Statemanager existiert.
        // Diese Überladung kann aufgrund mangelnder Parameter keinen Statemanager erstellen
        return nullptr;
    }
    return LifecycleStateManager::current_manager_;
}


std::shared_ptr<LifecycleStateManager> LifecycleStateManager::GetStateManager(std::unique_ptr<LifecycleState> initial_state, rclcpp::Node* current_node)
{
    if (!LifecycleStateManager::current_manager_) {
        // Wenn kein Statemanager existiert wird einer erstellt.
        LifecycleStateManager::current_manager_ = std::make_shared<LifecycleStateManager>(std::move(initial_state), current_node);
    }
    return LifecycleStateManager::current_manager_;
}

bool LifecycleStateManager::TransitionTo(std::unique_ptr<LifecycleState> new_state)
{
    RCLCPP_INFO(rclcpp::get_logger("LifecycleStateManager"), "Trying to queue transition from state '%s' to new state '%s'", this->current_state_->GetName().c_str(), new_state->GetName().c_str());
    // Wenn gerade kein Stateübergang ansteht, wird ein neuer Stateübergang eingerichtet, welcher beim nächste Lifecycleupdate durchgeführt wird.
    if (this->transition_ == nullptr) {
        RCLCPP_INFO(rclcpp::get_logger("LifecycleStateManager"), "Queued transition from state '%s' to new state '%s'", this->current_state_->GetName().c_str(), new_state->GetName().c_str());
        this->transition_ = std::move(new_state);
        return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("LifecycleStateManager"), "A transition was already queued", this->current_state_->GetName().c_str(), new_state->GetName().c_str());
    return false;
}

void LifecycleStateManager::Update()
{
    // Wenn ein Stateübergang ansteht, führe diesen durch
    if (this->transition_ != nullptr) {
        if (!this->HandleTransitionQueue()) {
            // TODO: Handle exception properly!
            RCLCPP_ERROR(rclcpp::get_logger("LifecycleStateManager"), "HandleTransitionQueue failed");
        }
    }

    // OnUpdate-Callback des aktuellen States aufrufen
    this->current_state_->OnUpdate();
}

bool LifecycleStateManager::HandleTransitionQueue()
{
    // Den anstehenden Stateübergang abrufen und entfernen
    std::unique_ptr<LifecycleState> new_state = std::move(this->transition_);
    this->transition_ = nullptr;

    // Der Stateübergang kann durchgeführt werden, wenn aktuell kein State gesetzt wurde oder wenn OnTransition des aktuellen States true zurückgibt
    if (this->current_state_ == nullptr || this->current_state_->OnTransition(new_state->GetName())) {
        this->current_state_ = std::move(new_state);

        // Übergang ist erfolgt, wenn OnBegin des neuen States true zurückgibt
        bool transition_successful = this->current_state_->OnBegin();

        if (transition_successful) {
            RCLCPP_INFO(rclcpp::get_logger("LifecycleStateManager"), "Transition to state '%s' successful", this->current_state_->GetName().c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("LifecycleStateManager"), "Transtition to state '%s' failed", this->current_state_->GetName().c_str());
        }
        return transition_successful;
    }
    RCLCPP_ERROR(rclcpp::get_logger("LifecycleStateManager"), "Transition from state '%s' to state '%s' forbidden", this->current_state_->GetName().c_str(), new_state->GetName().c_str());
    return false;
}

// Gibt einen Pointer auf diese Node zurück
rclcpp::Node* LifecycleStateManager::GetNode()
{
    return this->current_node_;
}
