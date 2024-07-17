#include <visually_dm/rviz_tools.hpp>


namespace rviz_tools
{

using namespace std::chrono_literals;

RvizTools::RvizTools(
    std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame)
    : node_(node)
    , base_frame_(base_frame)
    , display_timer_(node_->create_wall_timer(
        display_lifetime_
        , std::bind(&RvizTools::display, this)))
    , base_display_timer_(node_->create_wall_timer(
        2.0s
        , std::bind(&RvizTools::displayBase, this)))
    , int_server_("/avvv_interactives", node)
    , base_altitude_()
{
    
}

RvizTools::~RvizTools()
{
    
}

void RvizTools::addVehicle(
    const std::string& id
    , const std::string& colour
    , double opacity)
{
    if (obus_.find(id) == obus_.end()) {
        entities::Mesh new_vehicle{ 
            node_
            , base_frame_
            , entities::EntityType::vehicle
            , id
            , colour
            , 1.0
            , opacity
            , display_lifetime_.count() / display_lifetime_base_
        };

        obus_.emplace(id, new_vehicle);

        callbacks_.emplace(id, [this, id](interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
            obus_.at(id).toggleDisplayInfo();
        });

        int_server_.insert(new_vehicle.getInteractiveMarker());
        int_server_.setCallback(
            new_vehicle.getInteractiveMarker().name
            , callbacks_.at(id)
            , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
        
    }
}

void RvizTools::removeVehicle(
    const std::string& id)
{
    try {
        obus_.erase(id);
        int_server_.erase(id);
        
    }
    catch (std::out_of_range&) {}
}

void RvizTools::updateVehiclePose(
    const std::string& id
    , const geometry_msgs::msg::Pose& pose)
{
    try {
        auto& vehicle{ obus_.at(id) };
        vehicle.setPose(pose);
        updateLinkEndpoints(id, vehicle.getReceiverPoint());
        int_server_.setPose(id, pose);
        
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::updateVehicleEchoColour(
    const std::string& id
    , rviz_visual_tools::Colors colour)
{
    try {
        auto& vehicle{ obus_.at(id) };
        vehicle.setEchoColour(colour);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addRsu(
    const std::string& id
    , const std::string& colour
    , double opacity)
{
    if (rsus_.find(id) == rsus_.end()) {
        entities::Mesh new_rsu{
            node_
            , base_frame_
            , entities::EntityType::rsu
            , id
            , colour
            , 1.0
            , opacity
            , display_lifetime_.count() / display_lifetime_base_
        };
        
        rsus_.emplace(id, new_rsu);

        callbacks_.emplace(id, [this, id](interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
            rsus_.at(id).toggleDisplayInfo();
        });

        int_server_.insert(new_rsu.getInteractiveMarker());
        int_server_.setCallback(
            new_rsu.getInteractiveMarker().name
            , callbacks_.at(id)
            , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
        
    }
}

void RvizTools::removeRsu(
    const std::string& id)
{
    try {
        rsus_.erase(id);
        int_server_.erase(id);
        
    }
    catch (std::out_of_range&) {}
}

void RvizTools::updateRsuPose(
    const std::string& id
    , const geometry_msgs::msg::Pose& pose)
{
    try {
        auto& rsu{ rsus_.at(id) };
        rsu.setPose(pose);
        updateLinkEndpoints(id, rsu.getReceiverPoint());
        int_server_.setPose(id, pose);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addCloud(
    const std::string& id
    , const std::string& colour
    , double opacity)
{
    if (clouds_.find(id) == clouds_.end()) {
        entities::Mesh new_cloud{
            node_
            , base_frame_
            , entities::EntityType::cloud
            , id
            , colour
            , 1.0
            , opacity
            , display_lifetime_.count() / display_lifetime_base_
        };

        clouds_.emplace(id, new_cloud);

        callbacks_.emplace(id, [this, id](interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
            clouds_.at(id).toggleDisplayInfo();
        });

        int_server_.insert(new_cloud.getInteractiveMarker());
        int_server_.setCallback(
            new_cloud.getInteractiveMarker().name
            , callbacks_.at(id)
            , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
        
    }
}

void RvizTools::removeCloud(
    const std::string& id)
{
    try {
        clouds_.erase(id);
        int_server_.erase(id);
        
    }
    catch (std::out_of_range&) {}
}

void RvizTools::updateCloudPose(
    const std::string& id
    , const geometry_msgs::msg::Pose& pose)
{
    try {
        auto& cloud{ clouds_.at(id) };
        cloud.setPose(pose);
        updateLinkEndpoints(id, cloud.getReceiverPoint());
        int_server_.setPose(id, pose);        
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addSignalSet(
    const std::string& id
    , unsigned int crp_id)
{
    if (light_sets_.find(crp_id) == light_sets_.end()) {
        entities::LightSet new_light_set{
            node_
            , base_frame_
            , id
            , crp_id
        };

        light_sets_.emplace(crp_id, new_light_set);
    }
}

void RvizTools::addTrafficSignal(
    const std::string& id
    , unsigned int crp_id
    , const std::vector<unsigned int>& beacon_ids
    , const std::string& colour
    , const geometry_msgs::msg::Pose& pose
    , double opacity)
{
    entities::Light new_traffic_light{
        base_frame_
        , id
        , beacon_ids
        , colour
        , 1.0
        , opacity
        , entities::SignalType::traffic
    };

    new_traffic_light.setPose(pose);
    
    addSignalSet("Intersection_" + std::to_string(crp_id), crp_id);
    
    light_sets_.at(crp_id).addLight(new_traffic_light);

    callbacks_.emplace(
        id
        , [this, id, crp_id](
            interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
        light_sets_.at(crp_id).toggleDisplayInfo(id);
    });
    int_server_.insert(new_traffic_light.getInteractiveMarker());
    int_server_.setCallback(
        new_traffic_light.getInteractiveMarker().name
        , callbacks_.at(id)
        , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
    
}

void RvizTools::addPedestrianSignal(
    const std::string& id
    , unsigned int crp_id
    , const std::vector<unsigned int>& beacon_ids
    , const std::string& colour
    , const geometry_msgs::msg::Pose& pose
    , double opacity)
{
    entities::Light new_pedestrian_light{
        base_frame_
        , id
        , beacon_ids
        , colour
        , 1.0
        , opacity
        , entities::SignalType::pedestrian
    };

    new_pedestrian_light.setPose(pose);

    addSignalSet("Intersection_" + std::to_string(crp_id), crp_id);
    
    light_sets_.at(crp_id).addLight(new_pedestrian_light);

    callbacks_.emplace(
        id
        , [this, id, crp_id](
            interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
        light_sets_.at(crp_id).toggleDisplayInfo(id);
    });
    int_server_.insert(new_pedestrian_light.getInteractiveMarker());
    int_server_.setCallback(
        new_pedestrian_light.getInteractiveMarker().name
        , callbacks_.at(id)
        , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
    
}

void RvizTools::updateSignal(
    unsigned int crp_id
    , const dm_signal_info_msgs::msg::SignalInfo& signal_info)
{
    try {
        light_sets_.at(crp_id).updateSignalLight(signal_info);
    }
    catch (std::out_of_range&)
    { }
}

void RvizTools::addLink(const std::string& id, double max_dist)
{
    if (links_.find(id) == links_.end()) {
        entities::Link new_link{
            node_
            , base_frame_
            , id
            , max_dist
            , display_lifetime_.count() / display_lifetime_base_
        };

        links_.emplace(id, new_link);
    }
}

void RvizTools::updateLinkSpec(
    const std::string& id
    , rviz_visual_tools::Colors colour
    , double line_thickness
    , double opacity
    , double packet_dist)
{
    try {
        links_.at(id).updateLinkSpecs(colour, line_thickness, opacity, packet_dist);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addIndirectLink(
    const std::string& first_id
    , const std::string& second_id
    , const std::string& third_id)
{
    if (link_pairs_.find(first_id + second_id + third_id) == link_pairs_.end()) {
        entities::LinkPair new_link_pair{
            entities::Link(
                node_
                , base_frame_
                , first_id + second_id
                , std::numeric_limits<double>::max()
                , display_lifetime_.count() / display_lifetime_base_
            )
            , entities::Link(
                node_
                , base_frame_
                , second_id + third_id
                , std::numeric_limits<double>::max()
                , display_lifetime_.count() / display_lifetime_base_
            )
        };

        link_pairs_.emplace(first_id + second_id + third_id, new_link_pair);
    }
}

void RvizTools::updateIndirectLinkSpec(
    const std::string& id
    , rviz_visual_tools::Colors colour
    , double line_thickness
    , double opacity
    , double packet_dist)
{
    try {
        link_pairs_.at(id).rsu2cloud.updateLinkSpecs(colour, line_thickness, opacity, packet_dist);
        link_pairs_.at(id).cloud2vehicle.updateLinkSpecs(colour, line_thickness, opacity, packet_dist);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::activateLinks(
    std::vector<std::string> connected_link_ids)
{
    for (const auto& connected_link_id : connected_link_ids) {
        for (auto& link : links_)
            if (link.first == connected_link_id)
                link.second.activate();

        for (auto& link_pair : link_pairs_)
            if (link_pair.first == connected_link_id) {
                link_pair.second.rsu2cloud.activate();
                link_pair.second.cloud2vehicle.activate();
            }
    }
}

void RvizTools::addDetection(
    const std::string& id
    , const std::string& colour
    , int fade_time)
{
    if (detections_.find(id) == detections_.end()) {
        entities::Detection new_detection{ 
            node_
            , base_frame_
            , id
            , colour
            , fade_time
        };

        new_detection.setBaseAltitude(base_altitude_);

        detections_.emplace(id, new_detection);
    }
}

void RvizTools::updateDetection(
    const std::string& id)
{
    try {
        detections_.at(id).deleteDetections();
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::updateDetection(
    const std::string& id
    , const std::vector<dm_object_info_msgs::msg::ObjectInfo>& object_infos)
{
    try {
        detections_.at(id).addDetections(object_infos);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addFreespace(
    const std::string& id
    , const std::string& freespace_colour
    , double freespace_width)
{
    if (freespaces_.find(id) == freespaces_.end()) {
        entities::Freespace new_freespace{ 
            node_
            , base_frame_
            , id
            , freespace_colour
            , freespace_width
        };

        new_freespace.setBaseAltitude(base_altitude_);

        freespaces_.emplace(id, new_freespace);
    }
}

void RvizTools::updateFreespace(
    const std::string& id)
{
    try {
        freespaces_.at(id).deleteFreespaces();
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::updateFreespace(
    const std::string& id
    , const std::vector<dm_freespace_info_msgs::msg::FreespaceInfo>& freespace_infos)
{
    try {
        freespaces_.at(id).addFreespaces(freespace_infos);
    }
    catch (std::out_of_range&) {
    }
}


void RvizTools::setBaseAltitude(double base_altitude)
{
    base_altitude_ = base_altitude;
}


void RvizTools::display()
{
    for (auto& rsu : rsus_)
        rsu.second.publishUpdates(echo_points_);

    for (auto& obu : obus_) {
        obu.second.publishBaseUpdates(echo_points_);
        obu.second.publishUpdates(echo_points_);
    }

    for (auto& light_set : light_sets_)
        light_set.second.publishUpdates();

    for (auto& detection : detections_)
        detection.second.publishUpdates();

    for (auto& freespace : freespaces_)
        freespace.second.publishUpdates();

    for (auto& link : links_)
        link.second.publishUpdates();
    
    for (auto& link_pair : link_pairs_) {
        link_pair.second.rsu2cloud.publishUpdates();
        link_pair.second.cloud2vehicle.publishUpdates();
    }

    int_server_.applyChanges();
}

void RvizTools::displayBase()
{
    for (auto& rsu : rsus_)
        rsu.second.publishBaseUpdates(echo_points_);

    for (auto& cloud : clouds_)
        cloud.second.publishBaseUpdates(echo_points_);

    for (auto& light_set : light_sets_)
        light_set.second.publishBaseUpdates();
}

void RvizTools::updateLinkEndpoints(const std::string& id, const geometry_msgs::msg::Point& point)
{
    for (auto& link : links_)
        link.second.updateEndpoint(id, point);
    for (auto& link_pair : link_pairs_) {
        link_pair.second.rsu2cloud.updateEndpoint(id, point);
        link_pair.second.cloud2vehicle.updateEndpoint(id, point);
    }
}

} // namespace rviz_tools3