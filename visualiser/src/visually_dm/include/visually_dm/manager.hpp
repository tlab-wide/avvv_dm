/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Includes the main constants and functions
*/

#ifndef VISUALLY_DM__MANAGER_HPP_
#define VISUALLY_DM__MANAGER_HPP_

// C++
#include <memory>
#include <vector>
#include <utility>
#include <mutex>
#include <chrono>
#include <sstream>
#include <algorithm>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Package
#include <visually_dm/rviz_tools.hpp>
#include <visually_dm/transforms.hpp>
#include <visually_dm/net_status.hpp>
#include <dm_network_info_msgs/msg/object_info_n.hpp>
#include <dm_network_info_msgs/msg/signal_info_n.hpp>
#include <dm_network_info_msgs/msg/freespace_info_n.hpp>


namespace manager
{

// Configurations
const std::string rsu_colour{ "white" }; // Colour of the RSUs
const std::string obu_colour{ "white" }; // Colour of the OBUs
const std::string cloud_colour{ "white" }; // Colour of the Cloud servers
const std::string signal_colour{ "black" }; // Colour of the Signal lights

/**
 * @brief Handles the core logic of the application
*/
class Visualiser
{
public:
    /**
     * @brief Constructor
     * @param node
     * @param base_frame The reference frame used for visualisations, usually
     * 'world' or 'map'
     * @param rsu_obu_con_dist The maximum distance at which the RSUs and OBUs
     * stay visually connected
    */
    Visualiser(
        std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , net_status::NetworkField link_colour
        , net_status::NetworkField link_thickness
        , net_status::NetworkField link_packet_density
        , net_status::NetworkField link_opacity
        , const net_status::NetParamRanges& ranges
        , double rsu_obu_con_dist);

    /**
     * @brief Deconstructor
    */
    ~Visualiser();

    /**
     * @brief Adds a detection to the visualiser and subscribes to the relevant topic
     * to get updates related to RSUs
     * @param topic The topic on which the updates related to the detections comes from
     * @param detected_colour The colour the detections
    */
    void addRsuDetection(
        const std::string& topic
        , std::string detected_colour);

    /**
     * @brief Adds a detection to the visualiser and subscribes to the relevant topic
     * to get updates related to OBUs
     * @param topic The topic on which the updates related to the detections comes from
     * @param detected_colour The colour the detections
     * @param add_online_heatmap Whether or not to include online heatmaps processing
    */
    void addObuDetection(
        const std::string& topic
        , std::string detected_colour
        , bool add_online_heatmap);

    /**
     * @brief Adds a freespace to the visualiser and subscribes to the relevant topic
     * to get updates related to RSUs
     * @param topic The topic on which the freespace data comes in
     * @param freespace_colour The colour the freespace data gets visualised by
     * @param freespace_width The width of the line for the freespace
    */
    void addRsuFreespace(
        const std::string& topic
        , std::string freespace_colour
        , double freespace_width);

    /**
     * @brief Adds a freespace to the visualiser and subscribes to the relevant topic
     * to get updates related to OBUs
     * @param topic The topic on which the freespace data comes in
     * @param freespace_colour The colour the freespace data gets visualised by
     * @param freespace_width The width of the line for the freespace
     * @param add_online_heatmap Whether or not to include online heatmaps processing
    */
    void addObuFreespace(
        const std::string& topic
        , std::string freespace_colour
        , double freespace_width
        , bool add_online_heatmap);

    /**
     * @brief Adds RSUs to the visualiser
     * @param rsu_list The list of the RSU specifications
    */
    void addRsuList(
        const std::vector<std::string>& rsu_list);

    /**
     * @brief Adds OBUs to the visualiser and subscribes to the relevant topics to get updates
     * @param obu_list The list of the OBU specifications
     * @param obu_topics The list of the OBU TF topics
    */
    void addObuList(
        const std::vector<std::string>& obu_list
        , const std::vector<std::string>& obu_topics);

    /**
     * @brief Adds cloud servers to the visualiser
     * @param cloud The cloud server specifications
    */
    void addCloud(const std::string& cloud);

    /**
     * @brief Adds RSU-OBU or RSU-Cloud-OBU links to the visualiser
     * @param link_list The list of the link specifications containing
     * @param cloud_conf The configurations related to the cloud
     *  informations for cellular connections
     * the IDs of the entities
     * @note For correct programme behaviour, call this member function
     * before addRsuList, addObuList and addCloudList functions
    */
    void addLinkList(
        const std::vector<std::string>& link_list,
        const std::string& cloud_conf);

    /**
     * @brief Adds traffic signals to the visualiser and subscribes to the
     * relevant topic to get updates related to OBUs
     * @param signal_list The list of the signal specification
     * @param topic The signal_info topic that provides update data for the
     * traffic signals
     * @param add_online_heatmap Whether or not to include online heatmaps processing
    */
    void addObuSignalList(
        const std::vector<std::string>& signal_list
        , const std::vector<std::string>& topics
        , bool add_online_heatmap);

    /**
     * @brief Sets the base_altitude member attribute of the Rviz Tools
     * @param base_altitude
    */
    void setBaseAltitude(double base_altitude);

    /**
     * @brief Adds an offline heatmap type to art.
     * @param offline_heatmap_path The path to the CSV file containing heatmap data
     * @param rsu_id The ID of the RSU side of the heatmap
     * @param obu_id The ID of the OBU side of the heatmap
     * @param network_attr The network parameter which the heatmap represents
    */
    void addOfflineHeatmap(
        const std::string& offline_heatmap_path
        , const std::string& rsu_id
        , const std::string& obu_id
        , int network_attr);

private:
    /**
     * @brief Updates the position of the RSU with the given ID based on the
     * incoming TF message
     * @param msg
     * @param rsu_id
    */
    void rsuTfMessageCallback(
        const tf2_msgs::msg::TFMessage& msg
        , const std::string& rsu_id);

    /**
     * @brief Updates the position of the OBU with the given ID based on the
     * incoming TF message
     * @param msg
     * @param obu_id
    */
    void obuTfMessageCallback(
        const tf2_msgs::msg::TFMessage& msg
        , const std::string& obu_id);
    
    /**
     * @brief Displays the detected objects
     * @param msg
     * @param detection_id
    */
    void detectionMessageCallback(
        const dm_object_info_msgs::msg::ObjectInfoArray& msg
        , const std::string& detection_id);

    /**
     * @brief Displays the freespace
     * @param msg
     * @param freespace_id
    */
    void freespaceMessageCallback(
        const dm_freespace_info_msgs::msg::FreespaceInfoArray& msg
        , const std::string& freespace_id);

    /**
     * @brief Displays the signal data
     * @param msg
    */
    void signalMessageCallback(
        const dm_signal_info_msgs::msg::SignalInfoArray& msg);

    /**
     * @brief Update network informations
     * @param msg
    */
    void networkMessageCallback(
        const dm_network_info_msgs::msg::NetworkStatus& msg,
        const std::string& rsu_id,
        const std::string& obu_id,
        const std::string& protocol);

    /**
     * @brief Gets the list of the connected links
     * @param topic
     * @param ros_topic
     * @param connected_link_ids
    */
    void getConnectedLinks(
        const std::string& topic
        , std::string& ros_topic
        , std::vector<std::string>& connected_link_ids);

    // Link parameters represent parameters of the network status 
    net_status::NetworkField link_colour_;
    net_status::NetworkField link_thickness_;
    net_status::NetworkField link_packet_density_;
    net_status::NetworkField link_opacity_;

    // The ROS node
    std::shared_ptr<rclcpp::Node> node_;

    // The display-manager
    rviz_tools::RvizTools art_;

    // Used to prevent simultaneous access to art_ configurations and avoid race conditions
    std::mutex art_lock_;

    // The subscriptions on different topics and message types
    std::vector<std::shared_ptr<rclcpp::Subscription<dm_object_info_msgs::msg::ObjectInfoArray>>> object_info_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<dm_signal_info_msgs::msg::SignalInfoArray>>> signal_info_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<dm_freespace_info_msgs::msg::FreespaceInfoArray>>> freespace_info_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<dm_network_info_msgs::msg::ObjectInfoN>>> object_network_info_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<dm_network_info_msgs::msg::SignalInfoN>>> signal_network_info_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<dm_network_info_msgs::msg::FreespaceInfoN>>> freespace_network_info_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<dm_network_info_msgs::msg::NetworkStatus>>> network_status_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>>> tf_subscriptions_;
    
    net_status::NetStatusRepr net_status_repr_;

    double rsu_obu_con_dist_;

    const std::regex obu_rgx_{ "(OBU_\\d+)" };
    const std::regex link_rgx_{ "/(OBU_\\d+)/(RSU_\\d+)/([a-z]+_info)/network_status" };
};

} // namespace common

#endif // MANAGER_HPP_