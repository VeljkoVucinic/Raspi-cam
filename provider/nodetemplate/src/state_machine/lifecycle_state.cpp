//! Elternklasse! Nur zur Verwendung in Klassen, welche von dieser Klasse erben!

#include "nodetemplate/state_machine/lifecycle_state.hpp"

LifecycleState::LifecycleState() {}

// Da diese Klasse nicht direkt benutzt werden soll, gibt OnBegin false zurück, um eine Fehlermeldung zu zeigen
bool LifecycleState::OnBegin() { return false; }
bool LifecycleState::OnTransition(std::string _) { return false; }
void LifecycleState::OnError() {}

void LifecycleState::OnUpdate() {}

std::string LifecycleState::GetName()
{
    return typeid(this).name();
}

LifecycleState::~LifecycleState() {}