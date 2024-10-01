#include <visually_dm/entities.hpp>

namespace entities
{

EchoPoints::EchoPoints()
{
    Eigen::Vector3d rotated;
    double angle;
    constexpr double angle_inc{ M_PI * 2 / (echo_line_points - 1) };
    
    geometry_msgs::msg::Point point;
    std::size_t i, j;

    for (i = 0; i < max_echo_line_radius * 2; ++i) {
        Eigen::Vector3d base((i + 1) * 0.5, 0.0, 0.0);
        for (j = 0, angle = 0.0; j < echo_line_points; ++j, angle += angle_inc) {
            rotated = Eigen::Matrix3d(
                Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())) * base;
            point.x = rotated.x();
            point.y = rotated.y();
            point.z = rotated.z();
            points[i].push_back(point);
        }
    }
}

EchoPoints::~EchoPoints()
{

}

Entity::Entity(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& id
    , const std::string& base_frame)
    : id_(id)
    , rvizer_(rviz_visual_tools::RvizVisualTools(base_frame, id, node))
{
    rvizer_.setLifetime(0.0);
}

Entity::Entity(const Entity& other)
    : id_(other.id_)
    , rvizer_(other.rvizer_)
{
    
}

Entity::~Entity()
{
    rvizer_.deleteAllMarkers();
}

std::string Entity::getId()
{
    return id_;
}

void Entity::publishUpdates()
{

}

Mesh::Mesh(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , EntityType entity_type 
    , const std::string& id
    , const std::string& colour
    , double scale
    , double opacity
    , int moving_line_layer_update_rate
    , const std::string& echo_colour)
    : Entity(
        node
        , id
        , base_frame)
    , entity_type_(entity_type)
    , colour_(transforms::getRvizColour(colour))
    , echo_colour_(transforms::getRvizColour(echo_colour))
    , scale_(scale)
    , opacity_(opacity)
    , display_info_(false)
    , info_(id)
    , moving_line_layer_(0)
    , moving_line_layer_update_rate_(moving_line_layer_update_rate)
{
    initialise(base_frame);
    buildInteractiveMarker(base_frame);
    rvizer_.setAlpha(opacity_);
}

Mesh::~Mesh()
{

}

void Mesh::setPose(
    const double position[3]
    , const double orientation[3])
{
    /**
     * @todo Use Eigen for transformations
    */
    double trfmx[4][4];
    transforms::getTransformMatrix(trfmx, position, orientation);
    double receiver_point[3];
    transforms::transformPoint(receiver_point, relative_receiver_position_, trfmx);
    receiver_position_.x = receiver_point[0];
    receiver_position_.y = receiver_point[1];
    receiver_position_.z = receiver_point[2];

    text_pose_.position = receiver_position_;
    text_pose_.position.z += 2;    
}

void Mesh::setPose(const geometry_msgs::msg::Pose& pose)
{
    pose_ = pose;
    int_marker_.pose = pose;
    // Convert pose to straightforward arrays
    double position[3]{};
    double orientation[3]{};
    transforms::pose2Array(pose, position, orientation);
    setPose(position, orientation);
}

void Mesh::toggleDisplayInfo()
{
    display_info_ = not display_info_;
}

geometry_msgs::msg::Pose Mesh::getPose() const
{
    return pose_;
}

geometry_msgs::msg::Point Mesh::getReceiverPoint()
{
    return receiver_position_;
}

visualization_msgs::msg::InteractiveMarker Mesh::getInteractiveMarker()
{
    return int_marker_;
}

void Mesh::setEchoColour(rviz_visual_tools::Colors colour)
{
    echo_colour_ = colour;
}

void Mesh::setInfo(std::string& info)
{
    info_ = info;
}

void Mesh::initialise(const std::string& base_frame)
{
    auto file_path = "file://" + ament_index_cpp::get_package_share_directory("visually_dm");

    switch (entity_type_)
    {
    case EntityType::vehicle:
        model_file_ = file_path + "/meshes/car.obj";
        relative_receiver_position_[0] = 0.0;
        relative_receiver_position_[1] = 0.0;
        relative_receiver_position_[2] = 0.0;
        break;
    case EntityType::rsu:
        model_file_ = file_path + "/meshes/rsu.obj";
        relative_receiver_position_[0] = 0.0;
        relative_receiver_position_[1] = 0.0;
        relative_receiver_position_[2] = 8.0 * scale_;
        break;
    case EntityType::cloud:
        model_file_ = file_path + "/meshes/server.obj";
        relative_receiver_position_[0] = 0.0;
        relative_receiver_position_[1] = 2.5 * scale_;
        relative_receiver_position_[2] = 0.0;
        break;
    }

    line_1_.header.frame_id = line_2_.header.frame_id = line_3_.header.frame_id = base_frame;
    line_1_.ns = line_2_.ns = "spath"; line_3_.ns = "dpath";
    line_1_.id = 0; line_2_.id = 1; line_3_.id = 0;
    line_1_.type = line_2_.type = line_3_.type = line_1_.LINE_STRIP;
    line_1_.action = line_2_.action = line_3_.action = line_1_.ADD;
    line_1_.scale.x = line_1_.scale.y = line_1_.scale.z = 0.125;
    line_2_.scale.x = line_2_.scale.y = line_2_.scale.z = 0.125;
    line_3_.scale.x = line_3_.scale.y = line_3_.scale.z = 0.125;
}

void Mesh::buildInteractiveMarker(const std::string& base_frame)
{
    int_marker_.header.frame_id = base_frame;
    int_marker_.pose = pose_;
    int_marker_.scale = 2.0 * scale_;
    int_marker_.name = id_;

    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::SPHERE;
    box_marker.scale.x = 5.0;
    box_marker.scale.y = 5.0;
    box_marker.scale.z = 5.0;
    box_marker.color.a = 0.0;
    
    visualization_msgs::msg::InteractiveMarkerControl click_control;
    click_control.name = id_;
    click_control.always_visible = true;
    click_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    click_control.markers.push_back(box_marker);
    int_marker_.controls.push_back(click_control);
}

void Mesh::updateStaticEchoLines(const EchoPoints& echo_points)
{
    line_1_.points = echo_points.points[20];
    line_2_.points = echo_points.points[40];
    for (std::size_t i{}; i < echo_line_points; ++i) {

        line_1_.points[i].x += pose_.position.x;
        line_1_.points[i].y += pose_.position.y;
        line_1_.points[i].z += pose_.position.z;

        line_2_.points[i].x += pose_.position.x;
        line_2_.points[i].y += pose_.position.y;
        line_2_.points[i].z += pose_.position.z;
    }
}

void Mesh::updateMovingEchoLine(const EchoPoints& echo_points)
{
    line_3_.points = echo_points.points[moving_line_layer_ / 5];
    for (std::size_t i{}; i < echo_line_points; ++i) {
        
        line_3_.points[i].x += pose_.position.x;
        line_3_.points[i].y += pose_.position.y;
        line_3_.points[i].z += pose_.position.z;
    }
    moving_line_layer_ = (moving_line_layer_ + moving_line_layer_update_rate_) % (max_echo_line_radius * 10);
}

void Mesh::publishBaseUpdates(const EchoPoints& echo_points)
{
    rvizer_.publishMesh(pose_, model_file_, colour_, scale_, "mesh", 1);
    shape_msgs::msg::Mesh mesh;

    // Publish echo markers for entities other than traffic or pedestrian lights
    if (entity_type_ != EntityType::cloud) {
        updateStaticEchoLines(echo_points);
        line_1_.color = line_2_.color = rvizer_.getColor(echo_colour_);
        rvizer_.publishMarker(line_1_);
        rvizer_.publishMarker(line_2_);
    }

    if (entity_type_ == EntityType::cloud)
        rvizer_.trigger();
}

void Mesh::publishUpdates(const EchoPoints& echo_points)
{
    rvizer_.setLifetime(0.5);
    rvizer_.resetMarkerCounts();
    // Publish display text if applicable
    if (display_info_) {
        rvizer_.publishText(
            text_pose_
            , info_
            , rviz_visual_tools::Colors::WHITE
            , rviz_visual_tools::XXXXLARGE);
    }
    rvizer_.setLifetime(0.0);

    // Publish echo markers for entities other than traffic or pedestrian lights
    if (entity_type_ != EntityType::cloud) {
        updateMovingEchoLine(echo_points);
        line_3_.color = rvizer_.getColor(echo_colour_);
        rvizer_.publishMarker(line_3_);
    }

    rvizer_.trigger();
}


Light::Light(
    const std::string& base_frame
    , const std::string& id
    , const std::vector<unsigned int>& beacon_ids
    , const std::string& colour
    , double scale
    , double opacity
    , SignalType signal_type)
    : id_(id)
    , beacon_ids_(beacon_ids)
    , colour_(transforms::getRvizColour(colour))
    , scale_(scale)
    , opacity_(opacity)
    , signal_type_(signal_type)
    , display_info_(false)
    , info_(id)
{
    initialise();
    buildInteractiveMarker(base_frame);
}

Light::~Light()
{

}

void Light::setPose(
    const geometry_msgs::msg::Pose& pose)
{
    /**
     * @todo Use Eigen for transformations
    */
    pose_ = pose;

    double position[3];
    double orientation[3];

    transforms::pose2Array(pose, position, orientation);

    double trfmx[4][4];
    transforms::getTransformMatrix(trfmx, position, orientation);

    text_pose_ = pose_;
    text_pose_.position.z += 2;
    
    int_marker_.pose = pose;

    if (signal_type_ == SignalType::traffic) {
        double relative_light_point[3]{ 0.3, 1.2, 0.0 };

        double light_point[3]; 
        
        transforms::transformPoint(light_point, relative_light_point, trfmx);
        red_light_point_.x = light_point[0];
        red_light_point_.y = light_point[1];
        red_light_point_.z = light_point[2];

        relative_light_point[1] -= 2.4;
        transforms::transformPoint(light_point, relative_light_point, trfmx);
        green_light_point_.x = light_point[0];
        green_light_point_.y = light_point[1];
        green_light_point_.z = light_point[2];

        relative_light_point[1] += 1.2;
        transforms::transformPoint(light_point, relative_light_point, trfmx);
        yellow_light_point_.x = light_point[0];
        yellow_light_point_.y = light_point[1];
        yellow_light_point_.z = light_point[2];
    }
    else if (signal_type_ == SignalType::pedestrian) {
        double relative_light_point[3]{ 0.3, 0.0, -0.6 };

        double light_point[3]; 
        
        transforms::transformPoint(light_point, relative_light_point, trfmx);
        green_light_point_.x = light_point[0];
        green_light_point_.y = light_point[1];
        green_light_point_.z = light_point[2];

        relative_light_point[2] += 1.2;
        transforms::transformPoint(light_point, relative_light_point, trfmx);
        red_light_point_.x = light_point[0];
        red_light_point_.y = light_point[1];
        red_light_point_.z = light_point[2];
    }
}

void Light::toggleDisplayInfo()
{
    display_info_ = not display_info_;
}

visualization_msgs::msg::InteractiveMarker Light::getInteractiveMarker()
{
    return int_marker_;
}

void Light::setInfo(std::string& info)
{
    info_ = info;
}

void Light::updateSignalLight(
    const dm_signal_info_msgs::msg::SignalLightInfo& signal_light_info)
{
    using namespace dm_signal_info_msgs::msg;

    switch (signal_light_info.main_light.value)
    {
    case MainLightIndication::RED_LIGHT:
    case MainLightIndication::FLASHING_RED:
        state_ = TrafficLightState::red;
        break;
    case MainLightIndication::PERMISSIVE_YELLOW:
    case MainLightIndication::FLASHING_YELLOW:
        state_ = TrafficLightState::yellow;
        break;
    case MainLightIndication::PERMISSIVE_GREEN:
        state_ = TrafficLightState::green;
        break;
    default:
        state_ = TrafficLightState::off;
    }

    char buffer[32]{};

    snprintf(
        buffer
        , sizeof(buffer)
        , "%.2f"
        , signal_light_info.min_time_to_change.value
            * dm_standards::signal_time_coef);

    info_ = std::string(buffer);
}

void Light::initialise()
{
    auto file_path = "file://" + ament_index_cpp::get_package_share_directory("visually_dm");

    switch (signal_type_)
    {
    case SignalType::traffic:
        model_file_ = file_path + "/meshes/traffic_light.obj";
        break;
    case SignalType::pedestrian:
        model_file_ = file_path + "/meshes/pedestrian_light.obj";
        break;
    }
}

void Light::buildInteractiveMarker(
    const std::string& base_frame)
{
    int_marker_.header.frame_id = base_frame;
    int_marker_.pose = pose_;
    int_marker_.scale = 2.0 * scale_;
    int_marker_.name = id_;

    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::SPHERE;
    box_marker.scale.x = 5.0;
    box_marker.scale.y = 5.0;
    box_marker.scale.z = 5.0;
    box_marker.color.a = 0.0;
    
    visualization_msgs::msg::InteractiveMarkerControl click_control;
    click_control.name = id_;
    click_control.always_visible = true;
    click_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    click_control.markers.push_back(box_marker);
    int_marker_.controls.push_back(click_control);
}


LightSet::LightSet(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , const std::string& id
    , unsigned int crp_id)
    : Entity(
        node
        , id
        , base_frame)
    , id_(id)
    , crp_id_(crp_id)
{

}

LightSet::~LightSet()
{

}

void LightSet::addLight(const Light& light)
{
    for (const auto& current_light : lights_)
        if (current_light.id_ == light.id_)
            return;
    lights_.push_back(light);
}

void LightSet::updateSignalLight(
    const dm_signal_info_msgs::msg::SignalInfo& signal_info)
{
    // Extract the target beacon IDs
    std::vector<unsigned int> target_beacon_ids;
    for (const auto beacon_id : signal_info.signal_id_list)
        target_beacon_ids.push_back(beacon_id.value);

    std::sort(
        target_beacon_ids.begin()
        , target_beacon_ids.end());

    // Update lights whose beacon IDs match the target ones
    for (auto& light : lights_)
        if (light.beacon_ids_ == target_beacon_ids)
            light.updateSignalLight(signal_info.signal_light_info_list[0]);
}

void LightSet::toggleDisplayInfo(const std::string& id)
{
    for (auto& light : lights_)
        if (light.id_ == id)
            light.toggleDisplayInfo();
}

visualization_msgs::msg::InteractiveMarker LightSet::getInteractiveMarker(
    const std::string& id)
{
    visualization_msgs::msg::InteractiveMarker interactive_marker;
    for (auto& light : lights_)
        if (light.id_ == id)
            return light.getInteractiveMarker();
    return interactive_marker;
}

void LightSet::publishBaseUpdates()
{
    for (std::size_t i{}; i < lights_.size(); ++i)
        rvizer_.publishMesh(
            lights_[i].pose_
            , lights_[i].model_file_
            , lights_[i].colour_
            , lights_[i].scale_
            , "mesh"
            , i);

}

void LightSet::publishUpdates()
{
    rvizer_.setLifetime(0.2);
    
    for (const auto& light : lights_)
        // Publish display text if applicable
        if (light.display_info_) {
            rvizer_.publishText(
                light.text_pose_
                , light.info_
                , rviz_visual_tools::Colors::WHITE
                , rviz_visual_tools::XXXXLARGE
                , false);
        }
    
    rvizer_.setLifetime(0.0);

    static geometry_msgs::msg::Pose pose;
    for (std::size_t i{}; i < lights_.size(); ++i) {
        switch (lights_[i].state_)
        {
        case TrafficLightState::red:
            pose.position = lights_[i].red_light_point_;
            rvizer_.publishSphere(
                pose
                , rviz_visual_tools::RED
                , light_scale_
                , "light_spheres"
                , i);
            break;
        case TrafficLightState::yellow:
            pose.position = lights_[i].yellow_light_point_;
            rvizer_.publishSphere(
                pose
                , rviz_visual_tools::YELLOW
                , light_scale_
                , "light_spheres"
                , i);
            break;
        case TrafficLightState::green:
            pose.position = lights_[i].green_light_point_;
            rvizer_.publishSphere(
                pose
                , rviz_visual_tools::GREEN
                , light_scale_
                , "light_spheres"
                , i);
            break;
        default:
            rvizer_.deleteMarker("light_spheres", i);
            break;
        }

    }

    rvizer_.trigger();
}


Link::Link(
    const std::string& base_frame,
    const std::string& base_namespace,
    double max_dist)
    : valid_point_1_(false)
    , valid_point_2_(false)
    , base_namespace_(base_namespace)
    , max_dist_(max_dist)
    , counter_(0)
{
    line_.header.frame_id = base_frame;
    line_.type = line_.LINE_LIST;
    line_.action = line_.ADD;
    spheres_.header.frame_id = base_frame;
    spheres_.type = spheres_.SPHERE_LIST;
    spheres_.action = spheres_.ADD;
    spheres_.scale.x = packet_size;
    spheres_.scale.y = packet_size;
    spheres_.scale.z = packet_size;
}

Link::~Link()
{

}

void Link::updateEndpoint1(
    const geometry_msgs::msg::Point& point)
{
    point_1_ = point;
    valid_point_1_ = true;
}

void Link::updateEndpoint2(
    const geometry_msgs::msg::Point& point)
{
    point_2_ = point;
    valid_point_2_ = true;
}

std::vector<geometry_msgs::msg::Point> Link::getPacketPoints(
    double packet_dist,
    int counter_update_rate,
    int num_steps)
{
    const int num_packets = std::max(
        1,
        int (transforms::dist(point_1_, point_2_) / packet_dist));
    
    const double x_step = (point_2_.x - point_1_.x) / num_packets;
    const double y_step = (point_2_.y - point_1_.y) / num_packets;
    const double z_step = (point_2_.z - point_1_.z) / num_packets;

    const double pro = (double) counter_ / num_steps;
    const double reg = 1 - pro;
    
    std::vector<geometry_msgs::msg::Point> packet_points(2 * num_packets);
    
    packet_points[0].x = point_1_.x + pro * x_step;
    packet_points[0].y = point_1_.y + pro * y_step;
    packet_points[0].z = point_1_.z + pro * z_step;

    packet_points[1].x = point_1_.x + reg * x_step;
    packet_points[1].y = point_1_.y + reg * y_step;
    packet_points[1].z = point_1_.z + reg * z_step;

    for (int i{ 2 }; i < 2 * num_packets; ++i) {
        packet_points[i].x = packet_points[i - 2].x + x_step; 
        packet_points[i].y = packet_points[i - 2].y + y_step; 
        packet_points[i].z = packet_points[i - 2].z + z_step;
    }

    counter_ += counter_update_rate;
    counter_ %= num_steps;

    return packet_points;
}


LinkSet::LinkSet(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , double max_dist
    , const std::string& rsu_id
    , const std::string& obu_id
    , const std::string& cloud_id
    , int counter_update_rate)
    : Entity(node, rsu_id + obu_id, base_frame)
    , rsu_id_(rsu_id)
    , obu_id_(obu_id)
    , cld_id_(cloud_id)
    , rsu_obu_(Link(base_frame, "rsu_obu_", max_dist))
    , rsu_cld_(
        Link(
            base_frame,
            "rsu_cld_",
            std::numeric_limits<double>::max()))
    , cld_obu_(
        Link(
            base_frame,
            "cld_obu_",
            std::numeric_limits<double>::max()))
    , counter_update_rate_(counter_update_rate)
    , frame_id_(base_frame)
{

}

LinkSet::~LinkSet()
{

}

/**
 * @todo fix this about line and spheres
*/
void LinkSet::addProtocol(const std::string& protocol)
{
    link_infos_.emplace(protocol, LinkInformation{});
}

void LinkSet::updateRsuPoint(
    const std::string& rsu_id,
    const geometry_msgs::msg::Point& point)
{
    if (rsu_id == rsu_id_) {
        rsu_obu_.updateEndpoint1(point);
        if (cld_id_ != "")
            rsu_cld_.updateEndpoint1(point);
    }
}

void LinkSet::updateObuPoint(
    const std::string& obu_id
    , const geometry_msgs::msg::Point& point)
{
    if (obu_id == obu_id_) {
        rsu_obu_.updateEndpoint2(point);
        if (cld_id_ != "")
            cld_obu_.updateEndpoint2(point);
    }
}

void LinkSet::updateCloudPoint(
    const std::string& cloud_id
    , const geometry_msgs::msg::Point& point)
{
    if (cloud_id == cld_id_) {
        rsu_cld_.updateEndpoint2(point);
        cld_obu_.updateEndpoint1(point);
    }
}

void LinkSet::updateLinkSpecs(
    const std::string& protocol,
    rviz_visual_tools::Colors colour,
    double thickness,
    double opacity,
    double packet_dist)
{
    link_infos_.at(protocol).colour = colour;
    link_infos_.at(protocol).line_thickness = thickness;
    link_infos_.at(protocol).opacity = opacity;
    link_infos_.at(protocol).packet_dist = packet_dist;
}

void LinkSet::activate(const std::string& protocol)
{
    try {
        link_infos_.at(protocol).last_active_time = 
            std::chrono::steady_clock::now();
    }
    catch (std::out_of_range&) { }
}

void LinkSet::publishUpdates()
{
    publishLinkUpdate(rsu_obu_);
    if (cld_id_ != "") {
        publishLinkUpdate(rsu_cld_);
        publishLinkUpdate(cld_obu_);
    }
    rvizer_.trigger();
}

void LinkSet::publishLinkUpdate(Link& link)
{
    static bool valid_points;
    static bool active;
    static bool within_max_dist;
    valid_points = link.valid_point_1_ and link.valid_point_2_;
    within_max_dist = transforms::dist(link.point_1_, link.point_2_) <= link.max_dist_;

    if (valid_points) {
        link.line_.points.clear();
        link.line_.points.push_back(link.point_1_);
        link.line_.points.push_back(link.point_2_);
        for (const auto& link_info : link_infos_) {
            active = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - link_info.second.last_active_time)
                .count() < idle_time_;
            link.line_.ns = link.base_namespace_ + "line_" + link_info.first;
            link.spheres_.ns = link.base_namespace_ + "spheres_" + link_info.first;
            if (active and within_max_dist) {
                link.line_.color = rvizer_.getColor(link_info.second.colour);
                link.line_.color.a = link_info.second.opacity;
                link.line_.scale.x =
                    link.line_.scale.y =
                    link.line_.scale.z =
                    link_info.second.line_thickness;

                link.spheres_.points = link.getPacketPoints(
                    link_info.second.packet_dist,
                    counter_update_rate_,
                    num_steps_);
                link.spheres_.color = rvizer_.getColor(link_info.second.colour);
            }
            else {
                link.line_.color = rvizer_.getColor(rviz_visual_tools::WHITE);
                link.line_.color.a = pale_link_opacity;
                link.line_.scale.x =
                    link.line_.scale.y =
                    link.line_.scale.z = pale_link_thickness;

                link.spheres_.points.clear();
            }
            rvizer_.publishMarker(link.line_);
            rvizer_.publishMarker(link.spheres_);
        }
    }    
}


BoundingBox::BoundingBox(
    const double position[3]
    , const double yaw
    , const double dimension[3]
    , std::string object_class)
    : object_class_(object_class)
    , creation_time_(std::chrono::steady_clock::now())
{
    initialise(position, yaw, dimension);
}

BoundingBox::~BoundingBox()
{

}

void BoundingBox::initialise(
    const double position[3]
    , const double yaw
    , const double dimension[3])
{
    dimension_[0] = dimension[0];
    dimension_[1] = dimension[1];
    dimension_[2] = dimension[2];

    pose_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    pose_.translation() = Eigen::Vector3d(position[0], position[1], position[2] + dimension[2] / 2);
}


Detection::Detection(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , const std::string& id
    , const std::string& colour
    , int fade_time)
    : Entity(node, id, base_frame)
    , colour_(transforms::getRvizColour(colour))
    , fade_time_(fade_time)
    , fill_opacity_(0.2)
{

}

Detection::~Detection()
{

}

void Detection::deleteDetections()
{
    bboxes_.clear();
}

void Detection::addDetections(const std::vector<dm_object_info_msgs::msg::ObjectInfo>& object_infos)
{
    using namespace dm_object_info_msgs::msg;

    for (const auto& object_info : object_infos) {

        // If the location values are invalid, skip this object_info
        if (object_info.object_location.latitude.value == Latitude::UNKNOWN
                or object_info.object_location.longitude.value == Longitude::UNKNOWN)
            continue;
        
        double position[3];
        double dimension[3];
        
        transforms::toMgrs(
            object_info.object_location.longitude.value * dm_standards::location_coef
            , object_info.object_location.latitude.value * dm_standards::location_coef
            , position[0]
            , position[1]);

        
        if (object_info.object_location.altitude.value == Altitude::UNKNOWN)
            position[2] = base_altitude_;
        else
            position[2] = object_info.object_location.altitude.value * dm_standards::altitude_coef;

        
        double yaw{ (object_info.direction.value.value * dm_standards::angle_coef) };

        auto length{ object_info.size.length.value.value };
        auto width{ object_info.size.width.value.value };
        auto height{ object_info.size.height.value.value };

        std::string object_class;

        if (length == ObjectDimensionValue::UNKNOWN
                or width == ObjectDimensionValue::UNKNOWN
                or height == ObjectDimensionValue::UNKNOWN) {
            switch (object_info.object_class[getMaxClass(object_info.object_class)].id.value)
            {
            case ClassId::UNKNOWN:
                dimension[0]= 1.0;
                dimension[1]= 1.0;
                dimension[2]= 1.0;
                object_class = "UNKNOWN";
                break;
            case ClassId::VEHICLE:
                dimension[0] = dm_standards::vehicle_dim[0];
                dimension[1] = dm_standards::vehicle_dim[1];
                dimension[2] = dm_standards::vehicle_dim[2];
                object_class = "VEHICLE";
                break;
            case ClassId::PERSON:
                dimension[0] = dm_standards::person_dim[0];
                dimension[1] = dm_standards::person_dim[1];
                dimension[2] = dm_standards::person_dim[2];
                object_class = "PERSON";
                break;
            case ClassId::ANIMAL:
                dimension[0] = dm_standards::animal_dim[0];
                dimension[1] = dm_standards::animal_dim[1];
                dimension[2] = dm_standards::animal_dim[2];
                object_class = "ANIMAL";
                break;
            case ClassId::OTHER:
                dimension[0]= 1.0;
                dimension[1]= 1.0;
                dimension[2]= 1.0;
                object_class = "OTHER";
                break;
            }
        }
        else {
            dimension[0] = object_info.size.length.value.value * dm_standards::dimension_coef;
            dimension[1] = object_info.size.width.value.value * dm_standards::dimension_coef;
            dimension[2] = object_info.size.height.value.value * dm_standards::dimension_coef;
        }
        
        bboxes_.emplace_back(
            position
            , yaw
            , dimension
            , object_class);
    }
}

void Detection::publishUpdates()
{
    if (not bboxes_.size())
        return;

    rvizer_.resetMarkerCounts();
    rvizer_.deleteAllMarkers();
    for (const auto& bbox : bboxes_) {
        rvizer_.setAlpha(1.0);

        // Publish text above the bounding box
        rvizer_.setLifetime(0.5);
        rvizer_.publishText(
            bbox.pose_
            , bbox.object_class_
            , colour_
            , rviz_visual_tools::XXXXLARGE
            , false);
        rvizer_.setLifetime(0.0);

        rvizer_.publishWireframeCuboid(
            bbox.pose_
            , bbox.dimension_[1] // width = depth
            , bbox.dimension_[0] // length = width
            , bbox.dimension_[2]
            , colour_);
        rvizer_.setAlpha(fill_opacity_);
        rvizer_.publishCuboid(
            bbox.pose_
            , bbox.dimension_[1] // width = depth
            , bbox.dimension_[0] // length = width
            , bbox.dimension_[2]
            , colour_);
    }
    rvizer_.trigger();

    bboxes_.clear();
}

void Detection::setBaseAltitude(double base_altitude)
{
    base_altitude_ = base_altitude;
}

std::size_t Detection::getMaxClass(const std::vector<dm_object_info_msgs::msg::ObjectClass>& classes)
{
    auto size{ classes.size() };
    std::size_t max_index{};
    int max_class{};
    for (std::size_t i{}; i < size; ++i)
        if (classes[i].confidence.value > max_class) {
            max_class = classes[i].confidence.value;
            max_index = i;
        }
    return max_index;    
}


Segment::Segment(
    const double begin_position[3]
    , const double end_position[3]
    , double freespace_width)
{    
    initialise(begin_position, end_position, freespace_width);
}

Segment::~Segment()
{

}

void Segment::initialise(
    const double begin_position[3]
    , const double end_position[3]
    , double freespace_width)
{
    double yaw{ atan2(
        end_position[1] - begin_position[1]
        , end_position[0] - begin_position[0]) };

    Eigen::Vector3d rotated_forth;
    Eigen::Vector3d rotated_back;
    Eigen::Vector3d base(freespace_width / 2, 0.0, 0.0);
    geometry_msgs::msg::Point point;

    rotated_back = Eigen::Matrix3d(Eigen::AngleAxisd(yaw - M_PI_2, Eigen::Vector3d::UnitZ())) * base;
    rotated_forth = Eigen::Matrix3d(Eigen::AngleAxisd(yaw + M_PI_2, Eigen::Vector3d::UnitZ())) * base;
    
    point.x = begin_position[0] + rotated_back.x();
    point.y = begin_position[1] + rotated_back.y();
    point.z = begin_position[2] + rotated_back.z();
    points_.push_back(point);

    point.x = begin_position[0] + rotated_forth.x();
    point.y = begin_position[1] + rotated_forth.y();
    point.z = begin_position[2] + rotated_forth.z();
    points_.push_back(point);

    point.x = end_position[0] + rotated_back.x();
    point.y = end_position[1] + rotated_back.y();
    point.z = end_position[2] + rotated_back.z();
    points_.push_back(point);
    points_.push_back(point);

    point.x = end_position[0] + rotated_forth.x();
    point.y = end_position[1] + rotated_forth.y();
    point.z = end_position[2] + rotated_forth.z();
    points_.push_back(point);

    point.x = begin_position[0] + rotated_forth.x();
    point.y = begin_position[1] + rotated_forth.y();
    point.z = begin_position[2] + rotated_forth.z();
    points_.push_back(point);
}


Freespace::Freespace(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , const std::string& id
    , const std::string& freespace_colour
    , double freespace_width)
    : Entity(node, id, base_frame)
    , freespace_colour_(transforms::getRvizColour(freespace_colour))
    , freespace_width_(freespace_width)
{
    freespaces_.header.frame_id = base_frame;
    freespaces_.scale.x = freespaces_.scale.y = freespaces_.scale.z = 1.0;
    freespaces_.type = freespaces_.TRIANGLE_LIST;
    freespaces_.action = freespaces_.ADD;
    freespaces_.color = rvizer_.getColor(freespace_colour_);
}

Freespace::~Freespace()
{

}

void Freespace::deleteFreespaces()
{
    segments_.clear();
}

void Freespace::addFreespaces(const std::vector<dm_freespace_info_msgs::msg::FreespaceInfo>& freespace_infos)
{
    using namespace dm_object_info_msgs::msg;

    for (const auto& freespace_info : freespace_infos) {
        
        // If the location values are invalid, skip this freespace_info
        if (freespace_info.position_begin.latitude.value == Latitude::UNKNOWN
                or freespace_info.position_begin.longitude.value == Longitude::UNKNOWN
                or freespace_info.position_end.latitude.value == Latitude::UNKNOWN
                or freespace_info.position_end.longitude.value == Longitude::UNKNOWN)
            continue;

        double begin_position[3]{};
        double end_position[3]{};

        transforms::toMgrs(
            freespace_info.position_begin.longitude.value * dm_standards::location_coef
            , freespace_info.position_begin.latitude.value * dm_standards::location_coef
            , begin_position[0]
            , begin_position[1]);
        
        if (freespace_info.position_begin.altitude.value == Altitude::UNKNOWN)
            begin_position[2] = base_altitude_; 
        else
            begin_position[2] = freespace_info.position_begin.altitude.value * dm_standards::altitude_coef;

        transforms::toMgrs(
            freespace_info.position_end.longitude.value * dm_standards::location_coef
            , freespace_info.position_end.latitude.value * dm_standards::location_coef
            , end_position[0]
            , end_position[1]);

        if (freespace_info.position_end.altitude.value == Altitude::UNKNOWN)
            end_position[2] = base_altitude_;
        else
            end_position[2] = freespace_info.position_end.altitude.value * dm_standards::altitude_coef;

        segments_.emplace_back(
            begin_position
            , end_position
            , freespace_width_);
    }
}

void Freespace::publishUpdates()
{
    freespaces_.points.clear();
    freespaces_.colors.clear();

    for (auto& segment : segments_)
        freespaces_.points.insert(
            freespaces_.points.end()
            , segment.points_.begin()
            , segment.points_.end());

    for (std::size_t i{}; i < freespaces_.points.size(); ++i)
        freespaces_.colors.push_back(
            rvizer_.getColor(freespace_colour_));

    rvizer_.publishMarker(freespaces_);
    
    rvizer_.trigger();
}

void Freespace::setBaseAltitude(double base_altitude)
{
    base_altitude_ = base_altitude;
}

} // namespace entities