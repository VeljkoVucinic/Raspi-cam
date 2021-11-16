//RasPI provider
#include "nodetemplate/node_network/node_network_manager.hpp"

std::shared_ptr<NodeNetworkManager> NodeNetworkManager::current_manager_;

NodeNetworkManager::NodeNetworkManager()
{
    this->is_active_ = false;
}

std::shared_ptr<NodeNetworkManager> NodeNetworkManager::GetNodeNetworkManager()
{
    if (!NodeNetworkManager::current_manager_) {
        // Initialisiert neuen NodeNetworkManager, sollte es aktuell keinen geben.
        RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Initiating new node network manager");
        NodeNetworkManager::current_manager_ = std::make_shared<NodeNetworkManager>();
    }
    return NodeNetworkManager::current_manager_;
}

/*
static void NodeNetworkManager::requestComplete(Request *request, const std::map<Stream *, Buffer *> &buffers)
{
    RCLCPP_INFO(rclcpp::get_logger("RASPI_FRAME_THREAD"), "Request completed!");
}
*/

void NodeNetworkManager::heartbeat_publisher_timer_function()
{
    // Sende Heartbeat, wenn die Node aktiv ist
    if (this->is_active_) {
        auto message = std_msgs::msg::String();

        nlohmann::json heartbeat = {
            {"uuid", MetadataManager::GetMetadataManager()->GetUUID().str()}
        };

        message.data = heartbeat.dump();
        this->heartbeat_publisher_->publish(message);
    }
}

void NodeNetworkManager::pnp_heartbeat_subscriber_function(const std_msgs::msg::String::SharedPtr msg)
{
    // Wenn ein Heartbeat empfangen wurde, prüfe, ob die UUID einer aktuellen Abhängigkeit entspricht
    nlohmann::json message = nlohmann::json::parse(msg->data);
    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();
    std::vector<Dependency>* deps = mdm->GetDependencies();
    if (message["uuid"] != mdm->GetUUID().str()) {
        for (std::vector<Dependency>::iterator dep = deps->begin(); dep != deps->end(); ++dep) {
            if (dep->Satisfied != nullptr) {
                if (message["uuid"] == dep->Satisfied->UUID.str()) {
                    dep->Satisfied->LastBond = std::chrono::steady_clock::now();
                }
            }
        }
    }
}

void NodeNetworkManager::pnp_discovery_announce()
{
    // Sende eine Nachricht mit allen Capabilities und Metadaten dieser Node in den Plug and Play-Discovery Channel
    auto message = std_msgs::msg::String();
    
    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();

    nlohmann::json capabilities;
    for (std::vector<Capability>::iterator it = mdm->GetCapabilities()->begin(); it != mdm->GetCapabilities()->end(); ++it) {
        capabilities[it->Tag] = it->Error;
    }

    nlohmann::json content = {
        {"msg_type", "announce"},
        {"name", mdm->GetNodeName()},
        {"uuid", mdm->GetUUID().str()},
        {"capabilities", capabilities},
        {"meta_data", mdm->GetMetadata()},
    };

    message.data = content.dump();

    this->pnp_discovery_publisher_->publish(message);
}

void NodeNetworkManager::RefreshPnP()
{
    // Sendet eine "Such"-Nachricht nach allen Capabilities in den Plug and Play-Discovery-Channel
    this->pnp_discovery_search();
}

void NodeNetworkManager::AnnouncePnP()
{
    // Sendet ein Announcement in den Plug and Play-Discovery-Channel
    this->pnp_discovery_announce();
}

void NodeNetworkManager::Heartbeat()
{
    // Sendet einen Heartbeat
    this->heartbeat_publisher_timer_function();
}

void NodeNetworkManager::pnp_discovery_search()
{
    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();
    std::vector<Dependency>* deps = mdm->GetDependencies();

    // Sende eine Suche, nach allen noch nicht erfüllten Abhängigkeiten
    for (std::vector<Dependency>::iterator dep = deps->begin(); dep != deps->end(); ++dep) {
        if (dep->Satisfied == nullptr) {
            auto message = std_msgs::msg::String();
            
            nlohmann::json capabilities;
            

            nlohmann::json content = {
                {"msg_type", "search"},
                {"uuid", mdm->GetUUID().str()},
                {"capability", dep->Cap.Tag},
            };

            message.data = content.dump();

            this->pnp_discovery_publisher_->publish(message);
        }
    }
}

bool NodeNetworkManager::pnp_should_swap_dependency(std::vector<Dependency>::iterator dep)
{
    //!@ Füge hier eine Metrik ein, nach welcher entschieden wird, ob eine neue Abhängigkeit genommen wird
    //!@ Parameter dep: Abhängigkeit, um die es geht
    //!@ Parameter old_dependency: Die Node, mit welcher die Abhängigkeit aktuell erfüllt ist (kann auch ein leeres JSON-Objekt sein, wenn nicht belegt)
    //!@ Parameter new_dependency: Die Node, welche über PNP gefunden wurde
    //!@ Abhängigkeit wird ersetzt, wenn diese Funktion true zurückgibt
    sort(dep->Candidates.begin(), dep->Candidates.end(), [](const DependencyCandidate* lhs, const DependencyCandidate* rhs) -> bool {
        return lhs->Rating < rhs->Rating;
        });

    DependencyCandidate* BestCandidate = *(dep->Candidates.begin());

    if (dep->Satisfied == nullptr) {
        dep->Satisfied = BestCandidate;
        this->OnNewDependency(*dep);
        return true;
    }

    if (dep->Satisfied->UUID == BestCandidate->UUID) {
        return false;
    } else {
        dep->Satisfied = BestCandidate;
        this->OnNewDependency(*dep);
        return true;
    }
}

void NodeNetworkManager::OnNewDependency(Dependency changed_dependency) {
    //!@ Subscriptions u.ä. ändern
}


void NodeNetworkManager::pnp_discovery_subscriber_function(const std_msgs::msg::String::SharedPtr msg)
{
    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();
    std::vector<Dependency>* deps = mdm->GetDependencies();
    std::vector<Capability>* caps = mdm->GetCapabilities();


    // Verarbeite eine PNP-Discovery-Nachricht
    nlohmann::json message = nlohmann::json::parse(msg->data);
    if (message["uuid"] != mdm->GetUUID().str()) {
        if (message["msg_type"] == "announce") {
            // Prüfe, ob die Announce-Nachricht auf eine gewollte Abhängigkeit zutrifft
            for (std::vector<Dependency>::iterator dep = deps->begin(); dep != deps->end(); ++dep) {
                for (auto& capability : message["capabilities"].items()) {
                    if (dep->Cap.Tag == capability.key()) {
                        double OldRating = 0;
                        if (dep->Satisfied != nullptr) {
                            OldRating = dep->Satisfied->Rating;
                        }

                        DependencyCandidate* DiscoveredCandidate = nullptr;
                        for (std::vector<DependencyCandidate*>::iterator dep_cand = dep->Candidates.begin(); dep_cand != dep->Candidates.end(); dep_cand++) {
                            if ((*dep_cand)->UUID.str() == message["uuid"]) {
                                DiscoveredCandidate = *dep_cand;
                            }
                        }

                        if (DiscoveredCandidate == nullptr) {
                            DiscoveredCandidate = new DependencyCandidate();
                            DiscoveredCandidate->UUID = sole::rebuild(message["uuid"]);
                            DiscoveredCandidate->Name = message["name"];
                            DiscoveredCandidate->LastBond = std::chrono::steady_clock::now();
                            DiscoveredCandidate->MetaData = message["meta_data"];
                            DiscoveredCandidate->Error = short(capability.value());
                            DiscoveredCandidate->Rating = dep->CandidateRatingFunction(DiscoveredCandidate);

                            dep->Candidates.push_back(DiscoveredCandidate);
                        }
                        else {
                            DiscoveredCandidate->MetaData = message["meta_data"];
                            DiscoveredCandidate->Error = short(capability.value());
                            DiscoveredCandidate->Rating = dep->CandidateRatingFunction(DiscoveredCandidate);
                        }

                        if (this->pnp_should_swap_dependency(dep)) {
                            // Wenn die gefundene Node besser ist als die bestehende wird gewechselt.
                            RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "New node for '%s': %s", dep->InternalName.c_str(), dep->Satisfied->Name.c_str());
                            
                            this->OnNewDependency(*dep);
                            //!@ Hier Funktionen hinzufügen, welche beim Wechseln auf eine Abhängigkeit ausgeführt werden (z.B. Topics subscriben usw.)
                        } else {
                            //RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Won't change dependency for '%s' (New: %f <= Old: %f)", dep->InternalName.c_str(), DiscoveredCandidate->Rating, OldRating);
                        }
                    }
                }
            }
        } else if(message["msg_type"] == "search") {
            // Such-Nachrichten beantworten, wenn man die entsprechende Capability hat
            for (std::vector<Capability>::iterator it = caps->begin(); it != caps->end(); ++it) {
                if (it->Tag == message["capability"]) {
                    this->pnp_discovery_announce();
                    break;
                }
            }

        }
    }
}


bool NodeNetworkManager::InitiateEndpoints()
{
    // Die ROS2-Endpoints initialisieren

    rclcpp::Node* current_node = LifecycleStateManager::GetStateManager()->GetNode();

    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();
    
    // QoS-Profil, welches zuverlässige Nachrichtenübertragung gewährleistet, aber keine Nachrichten in der History speichert
    rmw_qos_profile_t pnp_profile = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            0,
            RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };

    // Heartbeat-Profil, welches zuverlässige Nachrichtenübertragung gewährleistet, aber keine Nachrichten in der History speichert
    rmw_qos_profile_t hb_profile = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            0,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };

    // Publishers
    // Plug and Play-Discovery
    this->pnp_discovery_publisher_ = current_node->create_publisher<std_msgs::msg::String>(
        "pnp_discovery",
        rclcpp::QoS(
            rclcpp::QoSInitialization(pnp_profile.history, pnp_profile.depth),
            pnp_profile
        )
    );

    // Heartbeat
    this->heartbeat_publisher_ = current_node->create_publisher<std_msgs::msg::String>(
        std::string("heartbeat"),
        rclcpp::QoS(
            rclcpp::QoSInitialization(hb_profile.history, hb_profile.depth),
            hb_profile
        )
    );
    
    //ovde
    this->raspi_publisher = current_node->create_publisher<std_msgs::msg::String>(
        std::string("raspi_" + mdm->GetSafeUUID()),
        1
    );

    //
    this->raspi_frame_thread = new std::thread([]() {
        RCLCPP_INFO(rclcpp::get_logger("RASPI_FRAME_THREAD"), "raspi_frame reader thread started!");

        //using namespace libcamera;
        //using namespace std; 

        std::shared_ptr<NodeNetworkManager> nnm = NodeNetworkManager::GetNodeNetworkManager();
        std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();

        //raspicam::RaspiCam Camera; //Camera object
        //nnm->Camera.setCaptureSize(640, 480);
        //nnm->Camera.setSaturation(100);
        //nnm->Camera.setFrameRate(5);

        usleep(3000 * 1000);
        if (! nnm->Camera.open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("RASPI_FRAME_THREAD"), "Pi Camera is not accessible yet...");
            usleep(300 * 1000);
        }
        //RCLCPP_INFO(rclcpp::get_logger("RASPI_FRAME_THREAD"), "Pi camera started!");
        usleep(3000*1000);        

        /*
        //libcamera
        shared_ptr<CameraManager> camera_manager
        shared_ptr<Camera> camera;
        unique_ptr<CameraConfiguration> config;

        camera_manager = new CameraManager();
        camera_manager->start();
        camera = camera_manager->cameras()[0];
        camera->acquire();

        //configuration
        config = camera->generateConfiguration({ StreamingRole::Viewfinder });
        StreamConfiguration& streamConfig = config->at(0);
        streamConfig.size.width = 640;
        streamConfig.size.height = 480;
        config->validate();
        RCLCPP_INFO(rclcpp::get_logger("RASPI_FRAME_THREAD"), "Default viewfinder configuration is: %s", streamConfig.toString());

        camera->configure(config.get());
        camera->allocateBuffers();

        vector<Request*> request;
        Stream* stream = streamConfig.stream();
        */

        
        Capability RaspiColorCapability = mdm->GetCapability("Color");
        RaspiColorCapability.Error = 1000;
        mdm->SetCapability(RaspiColorCapability);
        bool ShouldAnnounceRaspiColor = true;
        
        int i = 1;

        while (i) try
        {
            //getting data from the frame            
            nnm->Camera.grab();
            //allocate memory
            unsigned char* buffer = new unsigned char[nnm->Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)];
            //extract the image in rgb format
            nnm->Camera.retrieve(buffer, raspicam::RASPICAM_FORMAT_RGB); //get camera image
            auto data_size = nnm->Camera.getImageTypeSize(nnm->Camera.getFormat());
            int w = nnm->Camera.getWidth();
            int h = nnm->Camera.getHeight();


            //libcamera
            /*
            for (unsigned int i = 0; i < streamConfig.bufferCount; i++)
            {
                Request* r = camera->createRequest();
                unique_ptr<Buffer> b = stream->createBuffer(i);
                r->addBuffer(move(b));

                requests.push_back(r);
            }
            //checking if buffer is filed with data
            camera->requestCompleted.connect(requestComplete);
            camera->start();

            for (const auto& r : requests)
                camera->queueRequest(r);
            */

            std::string Image = base64_encode(buffer, data_size, false);
            nlohmann::json raspiData = { {"width", w}, {"height", h}, {"colorData", Image} };
            
            RCLCPP_INFO(rclcpp::get_logger("RASPI_FRAME_THREAD"), "Got raspi frame! Sending it... Width and heigth are %dx%d: %d", w, h, data_size);

            if (ShouldAnnounceRaspiColor) {
                NodeNetworkManager::GetNodeNetworkManager()->AnnouncePnP();
            }

            //calling function for sending depth info
            NodeNetworkManager::GetNodeNetworkManager()->SendFrame(raspiData); 
        }
        catch (...)
        {
            RCLCPP_INFO(rclcpp::get_logger("RASPI_FRAME_THREAD"), "... Did't work ...");
            //here camera could be stopped if needed
            //nnm->Camera.close();
        }
    }); //thread ending


    //!@ Hier können eigene Publisher hinzugefügt werden.
    //!@ Methodendokumentation: https://docs.ros2.org/beta3/api/rclcpp/classrclcpp_1_1node_1_1Node.html#a7a098357407370a57b7910f1b51af1cf
    //!@ Tutorial: https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/

    // Subscribers
    this->pnp_discovery_subscriber_ = current_node->create_subscription<std_msgs::msg::String>(
        "pnp_discovery",
        rclcpp::QoS(
            rclcpp::QoSInitialization(pnp_profile.history, pnp_profile.depth),
            pnp_profile
        ),
        std::bind(&NodeNetworkManager::pnp_discovery_subscriber_function, this, std::placeholders::_1)
    );

    this->heartbeat_subscriber_ = current_node->create_subscription<std_msgs::msg::String>(
        "heartbeat",
        rclcpp::QoS(
            rclcpp::QoSInitialization(hb_profile.history, hb_profile.depth),
            hb_profile
        ),
        std::bind(&NodeNetworkManager::pnp_heartbeat_subscriber_function, this, std::placeholders::_1)
    );
    

    // Services
    //!@ Hier können eigene Services hinzugefügt werden.
    //!@ Methodendokumentation: https://docs.ros2.org/beta3/api/rclcpp/classrclcpp_1_1node_1_1Node.html#a7a098357407370a57b7910f1b51af1cf

    // Warte, bis das Netzwerk Zeit hatte auf die neuen Endpoints zu reagieren
    int waiting = 1;
    RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManger"), "Waiting for pnp subscribers [%d]", waiting);
    
    while (current_node->count_subscribers("pnp_discovery") == 0) {
        RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManger"), "Waiting for pnp subscribers [%d]", waiting);
        waiting++;
        usleep(1000 * 1000);
    }

    // Such-Nachricht lossenden
    this->pnp_discovery_search();
    return true;
}

//here raspi cam frames are published
void NodeNetworkManager::SendFrame(nlohmann::json raspiData)
{
    auto message = std_msgs::msg::String();
    message.data = raspiData.dump();
    this->raspi_publisher->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("COLOR_FRAME_THREAD"), "Raspi frame sent!");

    //for validation
    std::ofstream outFile("raspi_frame1.ppm", std::ios::out | std::ios::binary);
    outFile << "P6\n" << 640 << " " << 480 << "\n255\n";
    outFile << base64_decode(raspiData["colorData"]);    
}

bool NodeNetworkManager::IsActive()
{
    return this->is_active_;
}

void NodeNetworkManager::SetActive(bool new_active_state)
{
    RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Current node is now %s", (new_active_state ? "active" : "idle"));

    if (new_active_state) {
        RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Announcing in PNP discovery topic");
        this->pnp_discovery_announce();
    }

    this->is_active_ = new_active_state;
}


NodeNetworkManager::~NodeNetworkManager() {}
