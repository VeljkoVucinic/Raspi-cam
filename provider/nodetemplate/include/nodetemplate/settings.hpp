//! Allgemeine Einstellungen dieser Node

#ifndef SETTINGS
#define SETTINGS

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "utilities/sole/sole.hpp"
#include "utilities/json/json.hpp"

#include <memory>

#include <vector>
#include <map>
#include <string>

/*!
    ActivationDependency Struct:
        InternalName: Interner Name, welcher nur zu Loggingzwecken genutzt wird
        Capability: Capability-Tag, welchen die Abhängigkeit haben muss (z.B. 'blinker', ...)
        MetaData: Zusatzinformation zu der Abhängigkeit (z.B. Optimalposition, Optimalauflösung, ...)
        Soft: Kann diese Node ohne diese Abhängigkeit aktiv sein?
        Satisfied: Interne Variable, welche mit einem leeren JSON-Objekt initialisiert werden soll. Wird vom NodeNetworkManager gefüllt.
*/
struct ActivationDependency {
    std::string InternalName;
    std::string Capability;
    nlohmann::json MetaData;
    bool Soft;
    nlohmann::json Satisfied;
};

class NodeSettings {
public:
    static sole::uuid UUID;
    static std::string SafeUUID();

    static std::string Name;

    static std::map<std::string, short> Capabilities;

    static nlohmann::json MetaData;

    static std::vector<ActivationDependency> ActivationDependencies;

    static short MaxHeartbeatMisses;
};

#endif