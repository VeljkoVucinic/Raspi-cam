#include "nodetemplate/settings.hpp"

//!@ Hier den Nodenamen ändern
std::string NodeSettings::Name = "node_name_here";

sole::uuid NodeSettings::UUID = sole::uuid4();

//!@ Hier die maximal verpassten Heartbeats einstellen
short NodeSettings::MaxHeartbeatMisses = 5;

//!@ Hier die Anforderungen einfügen
// Anforderungen für die Laufzeit der Node
std::vector<ActivationDependency> NodeSettings::ActivationDependencies {
/*    {   // Indicator - back left
        "indicator back left",  // Name for internal use
        "tag",                  // Tag
        {                       // Meta data
            {"location", {0, 0.5f, 0}},
            {"location_tolerance", {.5f, .5f, .5f}}
        },
        true,                   // Soft dependency
        nlohmann::json({})      // Dependency satisfied
    }, */
};

//!@ Hier die Capabilities dieser Node einfügen
// Die Capabilities der Node
std::map<std::string, short> NodeSettings::Capabilities {
/*
    std::make_pair<std::string, short>("capability_tag", [hier Fehleroffset einfügen]),
    -> der Fehleroffset wird mit dem Plug and Play-Announcement übertragen und soll einen
       Anhaltspunkt geben, "wie gut" diese Node diese Capability kann.
       Je niedriger die Zahl, umso besser.
*/
    std::make_pair<std::string, short>("pi_camera", 0),
};

// Diese Metadaten werden mit dem Plug and Play-Announcement übertragen
// und können benutzt werden, um zu entscheiden, ob die neue Node besser ist.
//!@ Hier Metadaten dieser Node ergänzen
nlohmann::json NodeSettings::MetaData = {
    // {"key", value},
    {"location", {0.5f, 0.5f, 0.5f}},
    {"saturation", 100}
};

// Diese Methode gibt die UUID der Node in einer Form zurück, welche den ROS2-Anforderungen genügt
std::string NodeSettings::SafeUUID()
{
    std::string safe_uuid = NodeSettings::UUID.str();
    while(safe_uuid.find("-") != std::string::npos) {
        safe_uuid.replace(safe_uuid.find("-"), 1, "_");
    }

    return safe_uuid;
}
