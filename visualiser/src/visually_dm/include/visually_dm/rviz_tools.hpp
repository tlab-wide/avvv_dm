/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Provides tools to simulate the entities and processes of an autonomous
 * vehicle network
*/

#ifndef VISUALLY_DM__RVIZ_TOOLS_HPP_
#define VISUALLY_DM__RVIZ_TOOLS_HPP_

// C++
#include <cmath>
#include <vector>
#include <string>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

// Rviz Visual Tools
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// Package
#include <visually_dm/entities.hpp>
#include <visually_dm/heatmap.hpp>
#include <dm_freespace_info_msgs/msg/freespace_info_array.hpp>
#include <dm_object_info_msgs/msg/object_info_array.hpp>
#include <dm_signal_info_msgs/msg/signal_info_array.hpp>


namespace rviz_tools
{

/**
 * @brief This enum is used for initialisation and indexing the responsible
 * rviz_visual_tool for different entities
*/
enum VisualManagers
{
    vehicles,
    rsus,
    clouds,
    connections,
    transmissions,
    bounding_boxes,
    off_heatmaps,
    on_heatmaps,
    max_num,
};

/**
 * @brief The chief class providing tools for simulations
 */
class RvizTools {
public:
    /**
     * @brief Constructor
     * @param node A ROS node
     * @param base_frame The reference frame used for visualisations, usually
     * 'world' or 'map'
     */
    RvizTools(
        std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame);

    /**
     * @brief Deconstructor
     */
    ~RvizTools();

    /**
     * @brief Adds a new vehicle to the network
     * @param id The name to assign to the vehicle
     * @param colour
     * @param opacity
     * @note The given ID must be unique, or the new vehicle won't be created
     */
    void addVehicle(
        const std::string& id
        , const std::string& colour
        , double opacity = 1.0);
    
    /**
     * @brief Removes a vehicle with the given ID
     * @param id
     * @note If the given ID is invalid or does not exist, this will be ignored
     */
    void removeVehicle(const std::string& id);
    
    /**
     * @brief Updates the pose of the vehicle with the given ID based on the
     * given pose
     * @param id
     * @param pose
     * @note If the given ID is invalid or does not exist, this will be ignored
     */
    void updateVehiclePose(
        const std::string& id
        , const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Updates vehicle's echo colour
     * @param id Vehicle's ID
     * @param colour The new colour to give
     * @note If the given ID is invalid or does not exist, this will be ignored
     */
    void updateVehicleEchoColour(
        const std::string& id
        , rviz_visual_tools::Colors colour);

    /**
     * @brief Adds a new RSU to the network
     * @param id The name to assign to the RSU
     * @param colour
     * @param opacity
     * @note The given ID must be unique, or the new RSU won't be created
     */
    void addRsu(
        const std::string& id
        , const std::string& colour
        , double opacity = 1.0);
    
    /**
     * @brief Removes an RSU with the given ID
     * @param id
     * @note If the given ID is invalid or does not exist, this will be ignored
     */
    void removeRsu(const std::string& id);
    
    /**
     * @brief Updates the pose of the RSU with the given ID based on the
     * given pose
     * @param id
     * @param pose
     * @note If the given ID is invalid or does not exist, this will be ignored
     */
    void updateRsuPose(
        const std::string& id
        , const geometry_msgs::msg::Pose& pose);
    
    /**
     * @brief Adds a new cloud server to the network
     * @param id The name to assign to the cloud
     * @param colour
     * @param opacity
     * @note The given ID must be unique, or the new cloud won't be created
     */
    void addCloud(
        const std::string& id
        , const std::string& colour
        , double opacity = 1.0);
    
    /**
     * @brief Removes a cloud with the given ID
     * @param id
     * @note If the given ID is invalid or does not exist, this will be ignored
     */
    void removeCloud(const std::string& id);
    
    /**
     * @brief Updates the pose of the Cloud server with the given ID based on the
     * given pose
     * @param id
     * @param pose
     * @note If the given ID is invalid or does not exist, this will be ignored
     */
    void updateCloudPose(
        const std::string& id
        , const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Adds a new traffic light
     * @param id
     * @param crp_id
     * @note The given ID must be unique, or this will be ignored
     */
    void addSignalSet(
        const std::string& id
        , unsigned int crp_id);
    
    /**
     * @brief Adds a new traffic light
     * @param id
     * @param crp_id
     * @param beacon_ids
     * @param colour
     * @param pose
     * @param opacity
     * @note The given ID must be unique, or this will be ignored
     */
    void addTrafficSignal(
        const std::string& id
        , unsigned int crp_id
        , const std::vector<unsigned int>& beacon_ids
        , const std::string& colour
        , const geometry_msgs::msg::Pose& pose
        , double opacity = 1.0);
    
    /**
     * @brief Adds a new pedestrian light
     * @param id
     * @param crp_id
     * @param beacon_ids
     * @param colour
     * @param pose
     * @param opacity
     * @note The given ID must be unique, or this will be ignored
     */
    void addPedestrianSignal(
        const std::string& id
        , unsigned int crp_id
        , const std::vector<unsigned int>& beacon_ids
        , const std::string& colour
        , const geometry_msgs::msg::Pose& pose
        , double opacity = 1.0);

    /**
     * @brief Adds several detection instances to the detection
     * @param crp_id
     * @param beacon_ids The IDs that determine which signal light or lights get affected by this update
     * @param signal_infos A signal message to get the signal specifications from
     * @note If the given ID is invalid, this will be ignored
     */
    void updateSignal(
        unsigned int crp_id
        , const dm_signal_info_msgs::msg::SignalInfo& signal_info);

    /**
     * @brief Adds a new link to the network
     * @param rsu_id
     * @param obu_id
     * @param protocol Including object_info, freespace_info and signal_info
     * @param max_dist The maximum distance at which the link stays visually connected
     * @param cloud_ids The list all cloud IDs that will later be added to the
     * visualiser
     * @note The given ID must be unique, or this will be ignored
     */
    void addLink(
        const std::string& rsu_id,
        const std::string& obu_id,
        const std::string& protocol,
        double max_dist,
        const std::string& cloud_id);

    /**
     * @brief Updates the features of the link corresponding to the given ID
     * @param rsu_id
     * @param obu_id
     * @param protocol Including object_info, freespace_info and signal_info
     * @param colour
     * @param line_thickness
     * @param opacity
     * @param packet_dist
     * @note If the given ID is invalid, this will be ignored
     */
    void updateLinkSpec(
        const std::string& rsu_id,
        const std::string& obu_id,
        const std::string& protocol,
        rviz_visual_tools::Colors colour,
        double line_thickness,
        double opacity,
        double packet_dist);

    /**
     * @brief Activates direct links as having a flow of data
     * @param rsu_id
     * @param obu_id
    */
    void activateLink(
        const std::string& rsu_id,
        const std::string& obu_id,
        const std::string& protocol);

    /**
     * @brief Adds a new detection
     * @param id
     * @param colour The colour of the bounding boxes of the detection
     * @param fade_time The time required for the instances to fully fade away
     * @note The given ID must be unique, or this will be ignored
     */
    void addDetection(
        const std::string& id
        , const std::string& colour
        , int fade_time = 0);

    /**
     * @brief Deletes all the instances from RViz
     * @param id
     * @note If the given ID is invalid, this will be ignored
     */
    void updateDetection(const std::string& id);

    /**
     * @brief Adds several detection instances to the detection
     * @param id
     * @param object_infos A list of object info messages to get the detection bounding box specifications from
     * @note If the given ID is invalid, this will be ignored
     */
    void updateDetection(
        const std::string& id
        , const std::vector<dm_object_info_msgs::msg::ObjectInfo>& object_infos);

    /**
     * @brief Adds a new freespace
     * @param id
     * @param freespace_colour The colour of the freespace line
     * @param freespace_width The width of the freespace line
     * @note The given ID must be unique, or this will be ignored
     */
    void addFreespace(
        const std::string& id
        , const std::string& freespace_colour
        , double freespace_width);

    /**
     * @brief Deletes all the instances from RViz
     * @param id
     * @note If the given ID is invalid, this will be ignored
     */
    void updateFreespace(const std::string& id);

    /**
     * @brief Adds several freespace areas to the freespace
     * @param id
     * @param freespace_infos A list of freespace info messages to get the freespace area specifications from
     * @note If the given ID is invalid, this will be ignored
     */
    void updateFreespace(
        const std::string& id
        , const std::vector<dm_freespace_info_msgs::msg::FreespaceInfo>& freespace_infos);

    /**
     * @brief Adds an offline heatmap recorder to the list of offline heatmaps
     * @param id
     * @param file_path Path to the CSV file containing heatmap data
     * @param net_status_repr The network status representative object
     * @param network_attr The network parameter the heatmap data is about
     * @note The given ID must be unique, or this will be ignored
    */
    void addOfflineHeatmap(
        const std::string& id
        , const std::string& file_path
        , const net_status::NetStatusRepr& net_status_repr
        , int network_attr);

    /**
     * @brief Adds an online heatmap recorder to the list of online heatmaps
     * @param id
     * @param net_status_repr The network status representative object
     * @param network_attr The network parameter the heatmap data is about
     * @note The given ID must be unique, or this will be ignored
    */
    void addOnlineHeatmap(
        const std::string& id
        , const net_status::NetStatusRepr& net_status_repr
        , int network_attr);

    /**
     * @brief Adds an online heatmap point to the specified online heatmap recorder
     * @param id
     * @param x
     * @param y
     * @param z
     * @param value
     * @note If the given ID is invalid, this will be ignored
    */
    void addToOnlineHeatmap(
        const std::string& id
        , const std::string& obu_id
        , double value);

    /**
     * @brief Sets the base_altitude member attribute
     * @param base_altitude
    */
    void setBaseAltitude(double base_altitude);

private:
    /**
     * @brief The callback for the timer to display the dynamic parts of all entities.
     * @note This method is by default called by a timer when an object of this class is created.
     * @note Without calling this method, nothing shows up on Rviz.
     */
    void display();

    /**
     * @brief The callback for the timer to display the static parts of all entities.
     * @note This method is by default called by a timer when an object of this class is created.
     * @note Without calling this method, nothing shows up on Rviz.
     */
    void displayBase();

    /**
     * @brief Updates all link endpoints position involving the
     * given RSU
     * @param rsu_id The ID of the RSU
     * @param point The new position of the endpoint
    */
    void updateLinkRsuPoints(
        const std::string& rsu_id
        , const geometry_msgs::msg::Point& point);

    /**
     * @brief Updates all link endpoints position involving the
     * given OBU
     * @param obu_id The ID of the OBU
     * @param point The new position of the endpoint
    */
    void updateLinkObuPoints(
        const std::string& obu_id
        , const geometry_msgs::msg::Point& point);

    /**
     * @brief Updates all link endpoints position involving the
     * given Cloud
     * @param cloud_id The ID of the Cloud
     * @param point The new position of the endpoint
    */
    void updateLinkCloudPoints(
        const std::string& cloud_id
        , const geometry_msgs::msg::Point& point);

    // ROS node to handle ROS specific tasks such as publishing, logging, etc
    std::shared_ptr<rclcpp::Node> node_;
    
    // The visualisation reference frame
    std::string base_frame_;
    
    // Maps and vectors of IDs to their corresponding objects    
    std::map<std::string, entities::Mesh> rsus_;
    std::map<std::string, entities::Mesh> obus_;
    std::map<std::string, entities::Mesh> clouds_;
    std::map<unsigned int, entities::LightSet> light_sets_;
    std::map<std::string, entities::Detection> detections_;
    std::map<std::string, entities::Freespace> freespaces_;
    std::map<std::string, entities::LinkSet> links_;

    std::map<std::string, heatmap::OfflineHeatmap> offline_heatmaps_;
    std::map<std::string, heatmap::OnlineHeatmap> online_heatmaps_;

    // The interactive marker callbacks for all the meshes (RSUs, OBUs and clouds)
    std::map<std::string, interactive_markers::InteractiveMarkerServer::FeedbackCallback> callbacks_;

    // Determines the sleep period (indirectly the "updates per second" rate)
    // Note that this value must never be greater that "display_lifetime_base"
    static constexpr std::chrono::milliseconds display_lifetime_{ 100 };
    static constexpr int display_lifetime_base_{ 10 };

    // The timer that updates RViz every 'display_lifetime' milliseconds
    rclcpp::TimerBase::SharedPtr display_timer_;
    rclcpp::TimerBase::SharedPtr base_display_timer_;
    
    // The interactive marker sever for interactive meshes
    interactive_markers::InteractiveMarkerServer int_server_;

    // The echo circle line points
    entities::EchoPoints echo_points_;
    
	// Base altitude for cases where the incoming DM messages
    // do not contain altitude data
    double base_altitude_;
};

} // namespace rviz_tools

#endif // RVIZ_TOOLS_HPP_