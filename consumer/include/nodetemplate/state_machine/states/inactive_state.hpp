#ifndef INACTIVE_STATE
#define INACTIVE_STATE

#include <cstdio>
#include <string>
#include <map>

#include "nodetemplate/metadata/metadata_manager.hpp"
#include "nodetemplate/state_machine/lifecycle_state.hpp"
#include "nodetemplate/state_machine/states/unconfigured_state.hpp"
#include "nodetemplate/state_machine/states/active_state.hpp"
#include "rclcpp/rclcpp.hpp"


class InactiveState : public LifecycleState
{
public:
    InactiveState();

    virtual bool OnBegin();
    virtual bool OnTransition(std::string);
    virtual void OnError();

    virtual void OnUpdate();

    virtual std::string GetName();

private:
    // Liste, aller States zu denen dieser State übergehen kann
    //!@ Hier alle möglichen nächsten States und Übergangsfunktionen einfügen
    const std::map<std::string, std::function<bool()>> allowed_transitions_ {
        std::make_pair<std::string, std::function<bool()>>("active_state", [this]() { return this->AreAllDependenciesSatisfied(); })
    };

    // Prüfen, ob alle Abhängigkeiten erfüllt sind
    bool AreAllDependenciesSatisfied();
};

#endif