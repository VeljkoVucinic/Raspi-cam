#include "nodetemplate/metadata/metadata_manager.hpp"

std::shared_ptr<MetadataManager> MetadataManager::current_manager_;

MetadataManager::MetadataManager(std::string node_name)
{
    // UUID generieren
    this->UUID = sole::uuid4();

    // Name setzen
    this->Name = node_name;

    // Restliche Variablen initialisieren
    this->Capabilities = std::vector<Capability>();
    this->Metadata = {};
    this->ActivationDependencies = std::vector<Dependency>();
}

MetadataManager::~MetadataManager() {}

std::shared_ptr<MetadataManager> MetadataManager::GetMetadataManager()
{
    if (!MetadataManager::current_manager_) {
        // Gibt einen Nullpointer zurück, wenn kein Statemanager existiert.
        // Diese Überladung kann aufgrund mangelnder Parameter keinen Statemanager erstellen
        return nullptr;
    }
    return MetadataManager::current_manager_;
}


std::shared_ptr<MetadataManager> MetadataManager::GetMetadataManager(std::string node_name)
{
    if (!MetadataManager::current_manager_) {
        // Wenn kein Statemanager existiert wird einer erstellt.
        MetadataManager::current_manager_ = std::make_shared<MetadataManager>(node_name);
    }
    return MetadataManager::current_manager_;
}

bool MetadataManager::IsConfigured()
{
    return this->Configured;
}

void MetadataManager::SetConfigured(bool configured)
{
    if (configured) {
        RCLCPP_INFO(rclcpp::get_logger("MetadataManager"), "Node set to CONFIGURED");
    }
    else {
        RCLCPP_INFO(rclcpp::get_logger("MetadataManager"), "Node set to UNCONFIGURED");
    }
    this->Configured = configured;
}

void MetadataManager::AddDependency(Dependency new_dependency)
{
    for (
        std::vector<Dependency>::iterator cur_dep_iter = this->ActivationDependencies.begin();
        cur_dep_iter < this->ActivationDependencies.end();
        ++cur_dep_iter
    ) {
        if (strcasecmp(cur_dep_iter->InternalName.c_str(), new_dependency.InternalName.c_str()) == 0) {
            return;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("MetadataManager"), "Added dependency '%s'", new_dependency.InternalName.c_str());

    this->ActivationDependencies.push_back(new_dependency);
}

bool MetadataManager::RemoveDependency(std::string remove_dependency)
{
    for (
        std::vector<Dependency>::iterator cur_dep_iter = this->ActivationDependencies.begin();
        cur_dep_iter < this->ActivationDependencies.end();
        ++cur_dep_iter
        ) {
        if (strcasecmp(cur_dep_iter->InternalName.c_str(), remove_dependency.c_str()) == 0) {
            this->ActivationDependencies.erase(cur_dep_iter);
            RCLCPP_INFO(rclcpp::get_logger("MetadataManager"), "Removed dependency '%s'", cur_dep_iter->InternalName.c_str());

            return true;
        }
    }
    return false;
}

std::vector<Dependency>* MetadataManager::GetDependencies()
{
    return &(this->ActivationDependencies);
}

std::vector<Capability>* MetadataManager::GetCapabilities()
{
    return &(this->Capabilities);
}

Capability MetadataManager::GetCapability(std::string capability)
{
    // Wenn Capability mit gegebenem Tag existiert, gebe Pointer darauf zurück
    for (
        std::vector<Capability>::iterator cur_cap_iter = this->Capabilities.begin();
        cur_cap_iter < this->Capabilities.end();
        ++cur_cap_iter
        ) {
        if (strcasecmp(cur_cap_iter->Tag.c_str(), capability.c_str()) == 0) {
            // Pointer auf das aktuell iterierte Objekt zurückgeben
            return (*cur_cap_iter);
        }
    }
    // Ansonsten Nullpointer zurückgeben
    Capability NoneCap = Capability();
    NoneCap.Tag = "TagNotFound";
    NoneCap.Error = 0;
    return NoneCap;
}

void MetadataManager::SetCapability(Capability new_capability)
{
    // Wenn Capability mit gegebenem Tag existiert, update nur den Fehleroffset
    for (
        std::vector<Capability>::iterator cur_cap_iter = this->Capabilities.begin();
        cur_cap_iter < this->Capabilities.end();
        ++cur_cap_iter
        ) {
        if (strcasecmp(cur_cap_iter->Tag.c_str(), new_capability.Tag.c_str()) == 0) {
            cur_cap_iter->Error = new_capability.Error;
            RCLCPP_INFO(rclcpp::get_logger("MetadataManager"), "Updated capability '%s' to %d", new_capability.Tag.c_str(), new_capability.Error);
            return;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("MetadataManager"), "Added capability '%s' (%d)", new_capability.Tag.c_str(), new_capability.Error);
    
    // Wenn die Capability noch nicht existiert, füge sie dem Array hinzu
    this->Capabilities.push_back(new_capability);
}

bool MetadataManager::RemoveCapability(std::string capability)
{
    for (
        std::vector<Capability>::iterator cur_cap_iter = this->Capabilities.begin();
        cur_cap_iter < this->Capabilities.end();
        ++cur_cap_iter
        ) {
        if (strcasecmp(cur_cap_iter->Tag.c_str(), capability.c_str()) == 0) {
            this->Capabilities.erase(cur_cap_iter);
            RCLCPP_INFO(rclcpp::get_logger("MetadataManager"), "Removed capability '%s'", capability.c_str());

            return true;
        }
    }

    return false;
}

sole::uuid MetadataManager::GetUUID()
{
    return this->UUID;
}

std::string MetadataManager::GetSafeUUID()
{
    std::string safe_uuid = this->GetUUID().str();
    while (safe_uuid.find("-") != std::string::npos) {
        safe_uuid.replace(safe_uuid.find("-"), 1, "_");
    }

    return safe_uuid;
}

std::string MetadataManager::GetNodeName()
{
    return this->Name;
}

std::string MetadataManager::GetFullNodeName()
{
    return this->Name + std::string("_") + this->GetSafeUUID();
}

nlohmann::json MetadataManager::GetMetadata()
{
    return this->Metadata;
}
