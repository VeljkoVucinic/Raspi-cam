//! MetadataManager

#ifndef METADATA_MANAGER
#define METADATA_MANAGER
#include "rclcpp/rclcpp.hpp"

#include "nodetemplate/utilities/sole/sole.hpp"
#include "nodetemplate/utilities/json/json.hpp"

#include <string.h>
#include <strings.h>
#include <map>
#include <vector>
#include <functional>
#include <chrono>


enum BondType {
    HEARTBEAT
};

struct Capability {
    std::string Tag;
    short Error;
};

struct DependencyCandidate {
    sole::uuid UUID;
    std::string Name;
    short Error;
    double Rating;
    std::chrono::time_point<std::chrono::steady_clock> LastBond;
    nlohmann::json MetaData;
};

struct Dependency {
    std::string InternalName;
    Capability Cap;
    bool Soft;

    BondType Bond;
    double BondRate;
    short MaxBondMisses;

    nlohmann::json MetaData;

    std::vector<DependencyCandidate*> Candidates;
    DependencyCandidate* Satisfied;

    std::function<double(DependencyCandidate*)> CandidateRatingFunction;
};

class MetadataManager
{
public:
    // Singleton Methoden
    MetadataManager(std::string);
    static std::shared_ptr<MetadataManager> GetMetadataManager();
    static std::shared_ptr<MetadataManager> GetMetadataManager(std::string);

    bool IsConfigured();
    void SetConfigured(bool);
    
    void AddDependency(Dependency);
    bool RemoveDependency(std::string);
    std::vector<Dependency>* GetDependencies();

    std::vector<Capability>* GetCapabilities();
    Capability  GetCapability(std::string);
    void        SetCapability(Capability);
    bool        RemoveCapability(std::string);

    sole::uuid  GetUUID();
    std::string GetSafeUUID();
    
    std::string GetNodeName();
    std::string GetFullNodeName();

    nlohmann::json GetMetadata();

    ~MetadataManager();
private:
    MetadataManager(const MetadataManager&);
    MetadataManager& operator=(const MetadataManager&);

    // Pointer auf wichtige Objekte
    static std::shared_ptr<MetadataManager> current_manager_;

    // Metadaten
    bool Configured = false;
    std::string Name;
    sole::uuid UUID;
    std::vector<Capability> Capabilities;
    nlohmann::json Metadata;
    std::vector<Dependency> ActivationDependencies;
};

#endif