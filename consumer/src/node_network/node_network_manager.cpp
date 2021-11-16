//consumer - Raspi
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
    nlohmann::json message = nlohmann::json::parse(msg->data); //here
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
    // In dieser Methode wird der beste Kandidat für die Abhängigkeit dep gefunden und falls notwendig dazu gewechselt
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
    }
    else {
        dep->Satisfied = BestCandidate;
        this->OnNewDependency(*dep);
        return true;
    }
}

void NodeNetworkManager::OnNewDependency(Dependency changed_dependency)
{
    //!@ Subscriptions u.ä. ändern 
    std::string safe_uuid = changed_dependency.Satisfied->UUID.str();
    while (safe_uuid.find("-") != std::string::npos)
    {
        safe_uuid.replace(safe_uuid.find("-"), 1, "_");
    }

    //usleep(2000 * 1000);
    //depth
    if (changed_dependency.Cap.Tag == "Depth") //metadata_manager.hpp
    {
        this->depth_subscriber = LifecycleStateManager::GetStateManager()->GetNode()->create_subscription<sensor_msgs::msg::Image>(
            std::string("depth_" + safe_uuid),
            1,
            std::bind(&NodeNetworkManager::depth_callback, this, std::placeholders::_1)
            );
        RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Depth callback should be called now");
    }


    //color
    else if (changed_dependency.Cap.Tag == "Color")
    {
        this->color_subscriber = LifecycleStateManager::GetStateManager()->GetNode()->create_subscription<std_msgs::msg::String>(
            std::string("color_" + safe_uuid),
            1,
            std::bind(&NodeNetworkManager::color_callback, this, std::placeholders::_1)
            );
    }

    //gyro
    else if (changed_dependency.Cap.Tag == "Gyro")
    {
        this->gyro_subscriber = LifecycleStateManager::GetStateManager()->GetNode()->create_subscription<sensor_msgs::msg::Imu>(
            std::string("gyro_" + safe_uuid),
            1,
            std::bind(&NodeNetworkManager::gyro_callback, this, std::placeholders::_1)
            );
        RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Gyro callback should be called now");
    }

    //accel
    else if (changed_dependency.Cap.Tag == "Accel")
    {
        this->accel_subscriber = LifecycleStateManager::GetStateManager()->GetNode()->create_subscription<sensor_msgs::msg::Imu>(
            std::string("accel_" + safe_uuid),
            1,
            std::bind(&NodeNetworkManager::accel_callback, this, std::placeholders::_1)
            );
    }

    RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Subscription changed to %s", std::string("Consumer_" + safe_uuid).c_str());
}



void NodeNetworkManager::pnp_discovery_subscriber_function(const std_msgs::msg::String::SharedPtr msg)
{
    std::shared_ptr<MetadataManager> mdm = MetadataManager::GetMetadataManager();
    std::vector<Dependency>* deps = mdm->GetDependencies();
    std::vector<Capability>* caps = mdm->GetCapabilities();


    // Verarbeite eine PNP-Discovery-Nachricht
    nlohmann::json message = nlohmann::json::parse(msg->data); //here
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
                        }
                        else {
                            //RCLCPP_INFO(rclcpp::get_logger("NodeNetworkManager"), "Won't change dependency for '%s' (New: %f <= Old: %f)", dep->InternalName.c_str(), DiscoveredCandidate->Rating, OldRating);
                        }
                    }
                }
            }
        }
        else if (message["msg_type"] == "search") {
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

//ovde
void NodeNetworkManager::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (this->is_active_ == true)
    {
        cv_bridge::CvImagePtr depth_bridge;
        depth_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth_mat = depth_bridge->image; //ovde
        int h_dep = depth_mat.size[0]; //width of frame
        int w_dep = depth_mat.size[1]; //height of frame
        //RCLCPP_INFO(rclcpp::get_logger("Depth-Callback"), "Dimensions of the frame %dX%d", w_dep, h_dep);

        //validation (prints average depth of center fo frame in meters)
        float sum = 0;
        int k = 0;
        for (int i = (h_dep / 2 - 5); i < (h_dep / 2 + 4); i++) //int i = 0; i < h; i++
        {
            for (int j = (w_dep / 2 - 6); j < (w_dep / 2 + 5); j++) //int j = 0; j < w; j++
            {
                sum += depth_mat.at<float>(i, j);
                if (depth_mat.at<float>(i, j) != 0)
                    k++;
            }
        }
        float center_depth = 111.222; //error value -> less then 0.4m or more than 20m
        if (k != 0)
            center_depth = sum / k; //k pixels average around center

        //nlohmann::json message = nlohmann::json::parse(msg->data);
        //float center_depth = message["depth"];
        //cv::Mat depth_mat = nlohmann::json::parse(msg->depth);
        //nlohmann::json depth_mat = nlohmann::json::parse(msg->depth);
        //int w = message["width"];
        //int h = message["height"];

        RCLCPP_INFO(rclcpp::get_logger("Depth-Callback"), "Depth of the center of frame: %.3fm", center_depth); //
    }

    /*
    if (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - this->last_echo).count() > 1) {
        RCLCPP_INFO(rclcpp::get_logger("Depth-Callback"), "Resolution of depth frame is: %dX%d. Depth of the center of frame: %f m, ", w, h, center_depth);
        this->last_echo = std::chrono::steady_clock::now();
    }
    */
}

void NodeNetworkManager::color_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (this->is_active_ == true)
    {
        nlohmann::json message = nlohmann::json::parse(msg->data);
        int w_col = message["width"];
        int h_col = message["height"];
        std::string colorData = base64_decode(message["colorData"]);

        /*
        If writting char array in .ppm file
        std::ofstream myfile;
        myfile.open("color_frame.ppm", std::ios::out | std::ios::binary);
        char header[] = { 'P', '6', '\n', '6', '4', '0', ' ', '4', '8', '0', '\n', '2', '5', '5', '\n' };
        myfile.write(header, 15);
        myfile.write(buffer, 3 * 480 * 640);
        myfile.close();
        */

        //Writting string in .ppm file
        std::ofstream outFile1("color_frame.ppm", std::ios::out | std::ios::binary);
        outFile1 << "P6\n" << w_col << " " << h_col << "\n255\n";
        outFile1 << colorData; //here

        RCLCPP_INFO(rclcpp::get_logger("Color-Callback"), "Color frame writing finished!");
    }
}


void NodeNetworkManager::gyro_callback(const sensor_msgs::msg::Imu::SharedPtr gyro_msg)
{
    if (this->is_active_ == true)
    {
        RCLCPP_INFO(rclcpp::get_logger("GYRO-Callback"), "Angular velocities: w_x =  %.3f, w_y = %.3f, w_z = %.3f", gyro_msg->angular_velocity.x, gyro_msg->angular_velocity.y, gyro_msg->angular_velocity.z);
    }
}


void NodeNetworkManager::accel_callback(const sensor_msgs::msg::Imu::SharedPtr accel_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Accel-Callback"), "Linear accelerations: a_x =  %.3f, a_y = %.3f, a_z = %.3f", accel_msg->linear_acceleration.x, accel_msg->linear_acceleration.y, accel_msg->linear_acceleration.z);
}


bool NodeNetworkManager::InitiateEndpoints()
{
    // Die ROS2-Endpoints initialisieren
    rclcpp::Node* current_node = LifecycleStateManager::GetStateManager()->GetNode();

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
