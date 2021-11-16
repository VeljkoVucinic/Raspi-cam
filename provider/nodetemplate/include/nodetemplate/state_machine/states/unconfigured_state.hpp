#pragma once

#include <cstdio>
#include <string>
#include <map>

#include "nodetemplate/state_machine/lifecycle_state.hpp"
#include "nodetemplate/state_machine/lifecycle_state_manager.hpp"

#include "nodetemplate/metadata/metadata_manager.hpp"

#include "nodetemplate/state_machine/states/inactive_state.hpp"

#include "nodetemplate/node_network/node_network_manager.hpp"

#include "rclcpp/rclcpp.hpp"
//ovde


class UnconfiguredState : public LifecycleState
{
public:
    UnconfiguredState();

    virtual bool OnBegin();
    virtual bool OnTransition(std::string);
    virtual void OnError();

    virtual void OnUpdate();

    virtual std::string GetName();

private:
    // Liste, aller States zu denen dieser State übergehen kann
    //!@ Hier alle möglichen nächsten States und Übergangsfunktionen einfügen
    const std::map<std::string, std::function<bool()>> allowed_transitions_ {
        // std::make_pair<std::string, std::function<bool()>>("state_name", [this]() { return false; /* Übergang möglich, wenn diese Funktion true zurückgibt */ }),
        std::make_pair<std::string, std::function<bool()>>("inactive_state", [this]() {
            return MetadataManager::GetMetadataManager()->IsConfigured();
         })
    };
};