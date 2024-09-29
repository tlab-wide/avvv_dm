#include <visually_dm/manager.hpp>


namespace manager
{


Visualiser::Visualiser(
    std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , net_status::NetworkField link_colour
    , net_status::NetworkField link_thickness
    , net_status::NetworkField link_packet_density
    , net_status::NetworkField link_opacity
    , const net_status::NetParamRanges& ranges
    , double rsu_obu_con_dist)
    : link_colour_(link_colour)
    , link_thickness_(link_thickness)
    , link_packet_density_(link_packet_density)
    , link_opacity_(link_opacity)
    , node_(node)
    , art_(node, base_frame)
    , net_status_repr_(ranges)
    , rsu_obu_con_dist_(rsu_obu_con_dist)
{

}

Visualiser::~Visualiser()
{

}

void Visualiser::addRsuDetection(
    const std::string& topic
    , std::string obu_detected_colour)
{
    auto detection_id{ topic + "/detections" };
    art_.addDetection(
        detection_id
        , detected_colour);
    
    object_info_subscriptions_.push_back(
        node_->create_subscription<dm_object_info_msgs::msg::ObjectInfoArray>(
            topic
            , 10
            , [this, detection_id, add_online_heatmap](
                    const dm_object_info_msgs::msg::ObjectInfoArray& msg) -> void {
                detectionMessageCallback(msg, detection_id);
            })
    );
}

void Visualiser::addObuDetection(
    const std::string& topic
    , std::string detected_colour
    , bool add_online_heatmap)
{
    auto detection_id{ topic + "/detections" };
    art_.addDetection(
        detection_id
        , detected_colour);
    
    std::smatch match_results;
    std::regex_search(topic, match_results, obu_rgx_);
    auto obu_id{ match_results[1].str() };
    if (add_online_heatmap) {
        art_.addOnlineHeatmap(
            topic + "heatmap/delay",
            net_status_repr_,
            0);
        art_.addOnlineHeatmap(
            topic + "heatmap/jitter",
            net_status_repr_,
            1);
        art_.addOnlineHeatmap(
            topic + "heatmap/rssi",
            net_status_repr_,
            2);
        art_.addOnlineHeatmap(
            topic + "heatmap/packet_loss",
            net_status_repr_,
            3);
    }

    object_network_info_subscriptions_.push_back(
        node_->create_subscription<dm_network_info_msgs::msg::ObjectInfoN>(
            topic
            , 10
            , [this, detection_id, add_online_heatmap, topic, obu_id](
                    const dm_network_info_msgs::msg::ObjectInfoN& msg) -> void {
                detectionMessageCallback(msg.object_info_array, detection_id);

                if (add_online_heatmap) {
                    art_.addToOnlineHeatmap(topic + "heatmap/delay", obu_id, msg.network_status.delay);
                    art_.addToOnlineHeatmap(topic + "heatmap/jitter", obu_id, msg.network_status.jitter);
                    art_.addToOnlineHeatmap(topic + "heatmap/rssi", obu_id, msg.network_status.rssi);
                    art_.addToOnlineHeatmap(topic + "heatmap/packet_loss", obu_id, msg.network_status.packet_loss);
                }
            })
    );
}

void Visualiser::addRsuFreespace(
    const std::string& topic
    , std::string freespace_colour
    , double freespace_width)
{
    auto freespace_id{ topic + "/freespaces" };
    art_.addFreespace(
        freespace_id
        , freespace_colour
        , freespace_width);
    freespace_info_subscriptions_.push_back(
        node_->create_subscription<dm_freespace_info_msgs::msg::FreespaceInfoArray>(
            topic
            , 10
            , [this, freespace_id](
                    const dm_freespace_info_msgs::msg::FreespaceInfoArray& msg) -> void {
                freespaceMessageCallback(msg, freespace_id);
            })
    );
}

void Visualiser::addObuFreespace(
    const std::string& topic
    , std::string freespace_colour
    , double freespace_width
    , bool add_online_heatmap)
{
    auto freespace_id{ topic + "/freespaces" };
    art_.addFreespace(
        freespace_id
        , freespace_colour
        , freespace_width);
    
    std::smatch match_results;
    std::regex_search(topic, match_results, obu_rgx_);
    auto obu_id{ match_results[1].str() };
    if (add_online_heatmap) {
        art_.addOnlineHeatmap(
            topic + "heatmap/delay",
            net_status_repr_,
            0);
        art_.addOnlineHeatmap(
            topic + "heatmap/jitter",
            net_status_repr_,
            1);
        art_.addOnlineHeatmap(
            topic + "heatmap/rssi",
            net_status_repr_,
            2);
        art_.addOnlineHeatmap(
            topic + "heatmap/packet_loss",
            net_status_repr_,
            3);
    }

    freespace_network_info_subscriptions_.push_back(
        node_->create_subscription<dm_network_info_msgs::msg::FreespaceInfoN>(
            topic
            , 10
            , [this, freespace_id, add_online_heatmap, topic, obu_id](
                    const dm_network_info_msgs::msg::FreespaceInfoN& msg) -> void {
                freespaceMessageCallback(msg.freespace_info_array, freespace_id);

                if (add_online_heatmap) {
                    art_.addToOnlineHeatmap(topic + "heatmap/delay", obu_id, msg.network_status.delay);
                    art_.addToOnlineHeatmap(topic + "heatmap/jitter", obu_id, msg.network_status.jitter);
                    art_.addToOnlineHeatmap(topic + "heatmap/rssi", obu_id, msg.network_status.rssi);
                    art_.addToOnlineHeatmap(topic + "heatmap/packet_loss", obu_id, msg.network_status.packet_loss);
                }
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

void Visualiser::addLinkList(const std::vector<std::string>& link_topics)
{
    std::smatch match_results;

    for (const auto& link_topic : link_topics) {
        std::regex_search(link_topic, match_results, link_rgx_);
        art_.addAllLinks(
            match_results[1].str(), // OBU ID
            match_results[2].str(), // RSU ID
            match_results[3].str(), // Protocol name (object, freespace or signal)
            rsu_obu_con_dist_);
        
        network_status_subscriptions_.push_back(
            node_->create_subscription<dm_network_info_msgs:msg::NetworkStatus>(
                link_topic
                , 10
                , [this, match_results](
                        const dm_network_info_msgs:msg::NetworkStatus& msg) -> void {
                    obuTfMessageCallback(msg, obu_id);
                }
            )
        );
    }
}

void Visualiser::addSignalList(
    const std::vector<std::string>& signal_list
    , const std::vector<std::string>& topics
    , bool add_online_heatmap)
{
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

    for (auto& topic : topics)
        signal_network_info_subscriptions_.push_back(
            node_->create_subscription<dm_network_info_msgs::msg::SignalInfoN>(
                topic
                , 10
                , [this](const dm_network_info_msgs::msg::SignalInfoN& msg) -> void {
                    signalMessageCallback(msg.signal_info_array);
                })
        );
}

void Visualiser::addOfflineHeatmap(
    const std::string& offline_heatmap_path
    , const std::string& rsu_id
    , const std::string& obu_id
    , int network_attr)
{
    std::string id{obu_id + '_' + rsu_id + "_offline_heatmap"};
    art_lock_.lock();
    art_.addOfflineHeatmap(
        id
        , offline_heatmap_path
        , net_status_repr_
        , network_attr);
    art_lock_.unlock();
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
    , const std::string& detection_id)
{
    art_.updateDetection(detection_id);
    art_.updateDetection(detection_id, msg.array);
}

void Visualiser::freespaceMessageCallback(
    const dm_freespace_info_msgs::msg::FreespaceInfoArray& msg
    , const std::string& freespace_id)
{
    art_.updateFreespace(freespace_id);
    art_.updateFreespace(freespace_id, msg.array);
}

void Visualiser::signalMessageCallback(
    const dm_signal_info_msgs::msg::SignalInfoArray& msg)
{
    for (const auto& signal_info : msg.array)
        art_.updateSignal(
            signal_info.crp_id.value
            , signal_info);
}

void Visualiser::networkMessageCallback(
    const dm_network_info_msgs::msg::NetworkStatus& msg,
    const std::string& rsu_id,
    const std::string& obu_id,
    const std::string& protocol)
{
    art_.updateAllLinkSpecs(
        rsu_id,
        obu_id,
        protocol,
        net_status_repr_.getColour[link_colour_](msg),
        net_status_repr_.getThickness[link_thickness_](msg),
        net_status_repr_.getOpacity[link_opacity_](msg),
        net_status_repr_.getPacketDensity[link_packet_density_](msg));
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