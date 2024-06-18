#include <visually_dm/manager.hpp>


namespace manager
{


Visualiser::Visualiser(
    std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , double rsu_obu_con_dist)
    : node_(node)
    , art_(node, base_frame)
    , rsu_obu_con_dist_(rsu_obu_con_dist)
{

}

Visualiser::~Visualiser()
{

}

void Visualiser::addDetection(
    const std::string& topic
    , std::string detected_colour)
{
    std::vector<std::string> connected_link_ids;
    std::string ros_topic;
    getConnectedLinks(topic, ros_topic, connected_link_ids);
    
    auto detection_id{ ros_topic + "/detections" };
    art_.addDetection(
        detection_id
        , detected_colour);
    
    object_info_subscriptions_.push_back(
        node_->create_subscription<dm_object_info_msgs::msg::ObjectInfoArray>(
            ros_topic
            , 10
            , [this, detection_id, connected_link_ids](const dm_object_info_msgs::msg::ObjectInfoArray& msg) -> void {
                detectionMessageCallback(msg, detection_id, connected_link_ids);
            })
    );
}

void Visualiser::addFreespace(
    const std::string& topic
    , std::string freespace_colour
    , double freespace_width)
{
    std::vector<std::string> connected_link_ids;
    std::string ros_topic;
    getConnectedLinks(topic, ros_topic, connected_link_ids);

    auto freespace_id{ ros_topic + "/freespaces" };
    art_.addFreespace(
        freespace_id
        , freespace_colour
        , freespace_width);
    freespace_info_subscriptions_.push_back(
        node_->create_subscription<dm_freespace_info_msgs::msg::FreespaceInfoArray>(
            ros_topic
            , 10
            , [this, freespace_id, connected_link_ids](const dm_freespace_info_msgs::msg::FreespaceInfoArray& msg) -> void {
                freespaceMessageCallback(msg, freespace_id, connected_link_ids);
            })
    );
}

void Visualiser::addRsuList(
    const std::vector<std::string>& rsu_list
    , const std::vector<std::string>& rsu_topics)
{
    assert(rsu_list.size() == rsu_topics.size()
        && "RSU list size not equal to RSU topic size.");
    
    std::string rsu_id;
    for (unsigned int i{}; i < rsu_list.size(); ++i) {
        std::istringstream iss(rsu_list[i]);
        iss >> rsu_id;
        art_.addRsu(rsu_id, rsu_colour);
        
        double position[3]{};
        double orientation[3]{};
        
        iss >> position[0];
        iss >> position[1];
        iss >> position[2];
        iss >> orientation[2];

        orientation[2] = transforms::degree2Radian(orientation[2]);
        geometry_msgs::msg::Pose initial_pose;
        transforms::getGeometryPose(position, orientation, initial_pose);
        
        art_.updateRsuPose(rsu_id, initial_pose);

        tf_subscriptions_.push_back(
            node_->create_subscription<tf2_msgs::msg::TFMessage>(
                rsu_topics[i]
                , 10
                , [this, rsu_id](const tf2_msgs::msg::TFMessage& msg) -> void {
                    rsuTfMessageCallback(msg, rsu_id);
                }
            )
        );
    }
}

void Visualiser::addObuList(
    const std::vector<std::string>& obu_list
    , const std::vector<std::string>& obu_topics)
{
    assert(obu_list.size() == obu_topics.size()
        && "OBU list size not equal to OBU topic size.");
    
    std::string obu_id;
    for (unsigned int i{}; i < obu_list.size(); ++i) {
        std::istringstream iss(obu_list[i]);
        iss >> obu_id;
        art_.addVehicle(obu_id, obu_colour);
        
        double position[3]{};
        double orientation[3]{};

        iss >> position[0];
        iss >> position[1];
        iss >> position[2];
        iss >> orientation[2];
        
        orientation[2] = transforms::degree2Radian(orientation[2]);        
        geometry_msgs::msg::Pose initial_pose;
        transforms::getGeometryPose(position, orientation, initial_pose);
        
        art_.updateVehiclePose(obu_id, initial_pose);

        tf_subscriptions_.push_back(
            node_->create_subscription<tf2_msgs::msg::TFMessage>(
                obu_topics[i]
                , 10
                , [this, obu_id](const tf2_msgs::msg::TFMessage& msg) -> void {
                    obuTfMessageCallback(msg, obu_id);
                }
            )
        );
    }
}

void Visualiser::addCloudList(const std::vector<std::string>& cloud_list)
{
    std::string cloud_id;
    for (const auto& cloud_conf : cloud_list) {
        std::istringstream iss(cloud_conf);
        iss >> cloud_id;
        art_.addCloud(cloud_id, cloud_colour);

        double position[3]{};
        double orientation[3]{};
        
        iss >> position[0];
        iss >> position[1];
        iss >> position[2];
        iss >> orientation[2];

        orientation[2] = transforms::degree2Radian(orientation[2]);
        geometry_msgs::msg::Pose initial_pose;
        transforms::getGeometryPose(position, orientation, initial_pose);

        art_.updateCloudPose(cloud_id, initial_pose);
    }
}

void Visualiser::addLinkList(const std::vector<std::string>& link_list)
{
    std::string first;
    std::string second;
    std::string third;
    for (const auto& link_conf : link_list) {
        std::istringstream iss(link_conf);
        iss >> first;
        iss >> second;
        iss >> third;
        if (third.size())
            art_.addIndirectLink(first, second, third);
        else
            art_.addLink(first + second, rsu_obu_con_dist_);
    }
}

void Visualiser::addSignalList(
    const std::vector<std::string>& signal_list
    , const std::string& topic)
{
    std::string ros_topic;
    std::vector<std::string> connected_link_ids;
    getConnectedLinks(topic, ros_topic, connected_link_ids);

    std::string signal_id;
    unsigned crp_id;
    std::string beacon_id;
    std::vector<unsigned int> beacon_ids;
    std::string signal_x;
    std::string signal_y;
    std::string signal_z;
    std::string signal_heading;
    for (const auto& signal_conf : signal_list) {
        std::istringstream iss(signal_conf);
        // Get the application-specific ID
        iss >> signal_id;

        // Get the CRP ID
        iss >> crp_id;

        // Get the beacon ID set
        beacon_ids.clear();
        iss >> beacon_id;
        while (beacon_id != "|") {
            beacon_ids.push_back(std::stoi(beacon_id));
            iss >> beacon_id;
        }

        std::sort(beacon_ids.begin(), beacon_ids.end());

        double position[3]{};
        double orientation[3]{};

        iss >> position[0];
        iss >> position[1];
        iss >> position[2];
        iss >> orientation[2];

        orientation[2] = transforms::degree2Radian(orientation[2]);
        geometry_msgs::msg::Pose initial_pose;
        transforms::getGeometryPose(position, orientation, initial_pose);
        
        if (signal_id[0] == 'P')
            art_.addPedestrianSignal(
                signal_id
                , crp_id
                , beacon_ids
                , signal_colour
                , initial_pose);
        
        else if (signal_id[0] == 'T')
            art_.addTrafficSignal(
                signal_id
                , crp_id
                , beacon_ids
                , signal_colour
                , initial_pose);
    }

    signal_info_subscriptions_.push_back(
        node_->create_subscription<dm_signal_info_msgs::msg::SignalInfoArray>(
            ros_topic
            , 10
            , [this, connected_link_ids](const dm_signal_info_msgs::msg::SignalInfoArray& msg) -> void {
                signalMessageCallback(msg, connected_link_ids);
            })
    );
}

void Visualiser::setBaseAltitude(double base_altitude)
{
    art_.setBaseAltitude(base_altitude);
}

void Visualiser::rsuTfMessageCallback(
    const tf2_msgs::msg::TFMessage& msg
    , const std::string& rsu_id)
{
    geometry_msgs::msg::Transform new_transform{ msg.transforms.at(0).transform };
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = new_transform.translation.x;
    new_pose.position.y = new_transform.translation.y;
    new_pose.position.z = new_transform.translation.z;
    new_pose.orientation = new_transform.rotation;
    art_lock_.lock();
    art_.updateRsuPose(rsu_id, new_pose);
    art_lock_.unlock();
}

void Visualiser::obuTfMessageCallback(
    const tf2_msgs::msg::TFMessage& msg
    , const std::string& obu_id)
{
    geometry_msgs::msg::Transform new_transform{ msg.transforms.at(0).transform };
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = new_transform.translation.x;
    new_pose.position.y = new_transform.translation.y;
    new_pose.position.z = new_transform.translation.z;
    new_pose.orientation = new_transform.rotation;
    art_lock_.lock();
    art_.updateVehiclePose(obu_id, new_pose);
    art_lock_.unlock();
}

void Visualiser::detectionMessageCallback(
    const dm_object_info_msgs::msg::ObjectInfoArray& msg
    , const std::string& detection_id
    , const std::vector<std::string> connected_link_ids)
{
    art_.updateDetection(detection_id);
    art_.updateDetection(detection_id, msg.array);
    art_.activateLinks(connected_link_ids);
}

void Visualiser::freespaceMessageCallback(
    const dm_freespace_info_msgs::msg::FreespaceInfoArray& msg
    , const std::string& freespace_id
    , const std::vector<std::string> connected_link_ids)
{
    art_.updateFreespace(freespace_id);
    art_.updateFreespace(freespace_id, msg.array);
    art_.activateLinks(connected_link_ids);
}

void Visualiser::signalMessageCallback(
    const dm_signal_info_msgs::msg::SignalInfoArray& msg
        , const std::vector<std::string> connected_link_ids)
{
    for (const auto& signal_info : msg.array) {
        art_.updateSignal(
            signal_info.crp_id.value
            , signal_info);
    }
    art_.activateLinks(connected_link_ids);
}

void Visualiser::getConnectedLinks(
    const std::string& topic
    , std::string& ros_topic
    , std::vector<std::string>& connected_link_ids)
{
    std::istringstream iss(topic);
    std::string connected_link_id;
    std::string temp;

    iss >> ros_topic;
    iss >> temp;
    if (temp == "[")
        while (temp != "]") {
            connected_link_id = "";
            iss >> connected_link_id;
            iss >> temp;
            connected_link_id += temp;
            iss >> temp;
            if (temp != "|" and temp != "]") {
                connected_link_id += temp;
                iss >> temp;
            }
            connected_link_ids.push_back(connected_link_id);
        }
}

} // namespace common