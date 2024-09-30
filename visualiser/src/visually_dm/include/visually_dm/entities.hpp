/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Includes the entities used in autonomous vehicle network visualisation
*/

#ifndef VISUALLY_DM__ENTITIES_HPP_
#define VISUALLY_DM__ENTITIES_HPP_

// C++
#include <string>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>
#include <mutex>
#include <cmath>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <shape_msgs/msg/mesh.hpp>

// Eigen
#include <Eigen/Dense>

// RViz Visual Tools
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// Package
#include <visually_dm/transforms.hpp>
#include <visually_dm/dm_standards.hpp>
#include <dm_freespace_info_msgs/msg/freespace_info_array.hpp>
#include <dm_object_info_msgs/msg/object_info_array.hpp>
#include <dm_signal_info_msgs/msg/signal_info_array.hpp>


namespace entities
{

// The time in milliseconds after which with no incoming updates, entities are considered idle
constexpr int idle_time_{ 200 };

// The number of points that make up the echo circle lines
constexpr std::size_t echo_line_points{ 100 };
// The max radius of the echo circle
constexpr int max_echo_line_radius{ 30 };


/**
 * @brief Distinguishes the types of meshes
*/
enum class EntityType
{

vehicle = 0,
rsu,
cloud,

};

/**
 * @brief Distinguishes the types of traffic signals
*/
enum class SignalType
{

traffic = 0,
pedestrian,

};

/**
 * @brief Represents the state of the traffic lights
*/
enum class TrafficLightState
{

off = 0,
red,
yellow,
green,

};

/**
 * @brief Holds the characteristics of a link
*/
struct LinkInformation
{
    rviz_visual_tools::Colors colour;

    double line_thickness;
    double opacity;
    double packet_dist;

    visualization_msgs::msg::Marker line;
    visualization_msgs::msg::Marker spheres;

    // Keeps track of the last active time
    std::chrono::time_point<std::chrono::steady_clock> last_active_time;
};


/**
 * @brief Stores the points for the echo circle lines for the RSUs/OBUs
*/
class EchoPoints {
public:
    /**
     * @brief Constructor
    */
    EchoPoints();
    
    /**
     * @brief Deconstructor
    */
    ~EchoPoints();

private:
    // Give the Mesh class direct access to points
    friend class Mesh;
    // The array of points in each layer
    std::vector<geometry_msgs::msg::Point> points[max_echo_line_radius * 2];
};


/**
 * @brief Base class for every entity
*/
class Entity {
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param id The ID of the entity
     * @param base_frame The base frame to publish everything in (usually 'map')
    */
    Entity(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& id
        , const std::string& base_frame);

    /**
     * @brief Copy constructor
     * @param other Another entity to build up from
    */
    Entity(const Entity& other);

    /**
     * @brief Destructor
    */
    ~Entity();

    /**
     * @brief Returns the entity's ID
    */
    std::string getId();

    /**
     * @brief Publishes the updates to RViz
    */
    virtual void publishUpdates();

protected:
    std::string id_;
    
    // The responsible RvizVisualTool
    rviz_visual_tools::RvizVisualTools rvizer_;
};


/**
 * @brief Represents an entity (vehicle, RSU, cloud, traffic or pedestrian light) on the Rviz simulator
 */
class Mesh : public Entity {
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param base_frame The reference frame used for visualisations, usually 'world' or 'map'
     * @param entity_type Deciding whether to create a vehicle, RSU or cloud
     * @param id The name of the mesh
     * @param colour Entity's colour
     * @param scale Entity's dimensions scale
     * @param opacity Entity's opacity
     * @param moving_line_layer_update_rate The rate at which the echo lines move
     * @param echo_colour The echo circle line colour
	*/
    Mesh(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , EntityType entity_type
        , const std::string& id
        , const std::string& colour
        , double scale
        , double opacity
        , int moving_line_layer_update_rate = 1
        , const std::string& echo_colour = "white");
    
    /**
     * @brief Deconstructor
	*/
    ~Mesh();

    /**
     * @brief Sets the pose of this entity
     * @note Adjusts the pose of the receiver point of this entity as well
     * @param pose
	*/
    void setPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Turns displaying information above the entity on or off
    */
    void toggleDisplayInfo();

    /**
     * @brief Sets the pose of this entity
     * @note Adjusts the pose of the receiver point of this entity as well
     * @param pose
	*/
    geometry_msgs::msg::Pose getPose() const;

    /**
     * @returns The receiver position of the entity
    */
    geometry_msgs::msg::Point getReceiverPoint();

    /**
     * @returns The interactive marker of this entity
    */
    visualization_msgs::msg::InteractiveMarker getInteractiveMarker();

    /**
     * @brief Sets the colour of the echo circle line
     * @param colour
    */
    void setEchoColour(const rviz_visual_tools::Colors colour);

    /**
     * @brief Sets the info for this mesh
     * @param info
    */
    void setInfo(std::string& info);

    /**
     * @brief Publishes the updates to RViz
     * @param echo_points The echo line points to use for the echo effect
    */
    void publishUpdates(const EchoPoints& echo_points);

    /**
     * @brief Publishes the base updates (the 3D mesh and if applicable static echo lines) to RViz
     * @param echo_points The echo line points to use for the echo effect
    */
    void publishBaseUpdates(const EchoPoints& echo_points);

private:
    /**
     * @brief Initialises the newly created mesh for the mesh file address and the type
     * @param base_frame
	*/
    void initialise(const std::string& base_frame);

    /**
     * @brief Creates the interactive marker associating with this entity
     * @param base_frame
	*/
    void buildInteractiveMarker(const std::string& base_frame);

    /**
     * @brief Updates the static echo lines (when position changes)
     * @param base_frame
    */
    void updateStaticEchoLines(const EchoPoints& echo_points);

    /**
     * @brief Updates the moving echo line as time goes by
     * @param base_frame
    */
    void updateMovingEchoLine(const EchoPoints& echo_points);

    /**
     * @brief Sets the pose of this entity
     * @param position
     * @param orientation
     * @note Adjusts the pose of the receiver point of this entity as well
	*/
    void setPose(
        const double position[3]
        , const double orientation[3]);

    EntityType entity_type_;
    geometry_msgs::msg::Pose pose_;

    // We need 2 receiver positions, one an array of doubles to hold the
    // relative position of the receiver with respect to the current pose,
    // another a geometry_msgs::msg::Point to contain the absolute position.
    double relative_receiver_position_[3];
    geometry_msgs::msg::Point receiver_position_;

    // The pose of the display text
    geometry_msgs::msg::Pose text_pose_;

    rviz_visual_tools::Colors colour_;
    rviz_visual_tools::Colors echo_colour_;

    double scale_;
    double opacity_;
    std::string model_file_;
    
    // If display_info_ is set to 'true' an overlay text of the information
    // about this entity is displayed in the Rviz simulator
    bool display_info_;
    std::string info_;

    // The interactive marker for this entity so that the user can click on the entity
    visualization_msgs::msg::InteractiveMarker int_marker_;

    // The echo circles
    visualization_msgs::msg::Marker line_1_;
    visualization_msgs::msg::Marker line_2_;
    visualization_msgs::msg::Marker line_3_;
    int moving_line_layer_;
    int moving_line_layer_update_rate_;
};


/**
 * @brief Represents either a traffic or a pedestrian beacon on the Rviz simulator
 */
class Light {
public:
    /**
     * @brief Constructor
     * @param base_frame The base map
     * @param id The conventional ID of this light
     * @param beacon_ids The beacon IDs representing this light
     * @param colour Light's colour
     * @param scale Light's dimensions scale
     * @param opacity Light's opacity
     * @param signal_type Light's type (whether traffic or pedestrian)
	*/
    Light(
        const std::string& base_frame
        , const std::string& id
        , const std::vector<unsigned int>& beacon_ids
        , const std::string& colour
        , double scale
        , double opacity
        , SignalType signal_type);
    
    /**
     * @brief Deconstructor
	*/
    ~Light();
    
    /**
     * @brief Sets the pose of this entity
     * \note Adjusts the pose of the receiver point of this entity as well
     * \param pose
	*/
    void setPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Turns displaying information above the entity on or off
    */
    void toggleDisplayInfo();

    /**
     * @returns The interactive marker of this entity
    */
    visualization_msgs::msg::InteractiveMarker getInteractiveMarker();

    /**
     * @brief Sets the info for this mesh
     * @param info
    */
    void setInfo(std::string& info);

    /**
     * @brief Updates the traffic light info (only if this mesh is of type traffic light)
     * @param signal_light_info
	*/
   	void updateSignalLight(
        const dm_signal_info_msgs::msg::SignalLightInfo& signal_light_info);

private:
    friend class LightSet;

    /**
     * @brief Initialises the newly created mesh for the mesh file address and the type
	*/
    void initialise();

    /**
     * @brief Creates the interactive marker associating with this entity
     * @param base_frame The base map
	*/
    void buildInteractiveMarker(
        const std::string& base_frame);

    std::string id_;

    geometry_msgs::msg::Pose pose_;

    // The pose of the display text
    geometry_msgs::msg::Pose text_pose_;

    std::vector<unsigned int> beacon_ids_;

    rviz_visual_tools::Colors colour_;

    double scale_;
    double opacity_;
    
    SignalType signal_type_;
    
    std::string model_file_;
    
    // If display_info_ is set to 'true' an overlay text of the information
    // about this entity is displayed in the Rviz simulator
    bool display_info_;
    std::string info_;

    // The interactive marker for this entity so that the user can click on the entity
    visualization_msgs::msg::InteractiveMarker int_marker_;

    // For traffic or pedestrian lights only
    TrafficLightState state_;
    geometry_msgs::msg::Point red_light_point_;
    geometry_msgs::msg::Point yellow_light_point_;
    geometry_msgs::msg::Point green_light_point_;

};


/**
 * @brief Represents an entity (vehicle, RSU, cloud, traffic or pedestrian light) on the Rviz simulator
 */
class LightSet : public Entity {
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param base_frame The reference frame used for visualisations, usually 'world' or 'map'
     * @param id The ID of the light set to use in Rviz
     * @param crp_id The CRP ID of the intersection this light set is in
	*/
    LightSet(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , const std::string& id
        , unsigned int crp_id);
    
    /**
     * @brief Deconstructor
	*/
    ~LightSet();

    /**
     * @brief Add a new light to the list of lights
     * @param light
    */
    void addLight(const Light& light);

    /**
     * @brief Updates the traffic light info (only if this mesh is of type traffic light)
     * @param signal_info
	*/
   	void updateSignalLight(
        const dm_signal_info_msgs::msg::SignalInfo& signal_info);

    /**
     * @brief Sets the text display on/off
     * @param id ID of the target signal light
    */
    void toggleDisplayInfo(const std::string& id);
    
    /**
     * @param id ID of the target signal light
     * @returns The interactive marker of a light signal
    */
    visualization_msgs::msg::InteractiveMarker getInteractiveMarker(
        const std::string& id);

    /**
     * @brief Publishes the updates to RViz
    */
    void publishUpdates();

    /**
     * @brief Publishes the base updates (the 3D mesh and if applicable static echo lines) to RViz
    */
    void publishBaseUpdates();

private:
    std::string id_;

    unsigned int crp_id_;

    std::vector<Light> lights_;

    double light_scale_{ 1.0 };

};


/**
 * @brief Represents a link (combination of a connection and its packet transmission current)
 */
class Link : public Entity {
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param base_frame The reference frame used for visualisations, usually "/world" or "/odom"
     * @param id The ID of the link
     * @param max_dist The maximum distance the link should stay connected
     * @param counter_update_rate The rate at which packets move on the link
     */
    Link(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , const std::string& id
        , const double max_dist
        , int counter_update_rate = 1);

    /**
     * @brief Deconstructor
     */
    ~Link();
    
    /**
     * @brief Update the position of the specified end point
     * @param protocol The protocol name (object, freespace or signal)
    */
    void addProtocol(const std::string& protocol);

    /**
     * @brief Update the position of the specified end point
     * @param end_point_id The ID of the entity this link meets at any end
     * @param point The new position of the end point
    */
    void updateEndpoint(
        const std::string& end_point_id
        , const geometry_msgs::msg::Point& point);

    /**
     * @brief Update the specifications of the link
     * @param protocol The protocol name (object, freespace or signal)
     * @param colour
     * @param thickness
     * @param opacity
     * @param packet_dist The new distance between any two packets
    */
    void updateLinkSpecs(
        const std::string& protocol,
        rviz_visual_tools::Colors colour,
        double thickness,
        double opacity,
        double packet_dist);

    /**
     * @brief Activates links as having a flow of data
    */
    void activate(const std::string& protocol);

    /**
     * @brief Publishes the updates to RViz
    */
    void publishUpdates();

private:
    /**
     * @brief Updates the position of the packets as if in a current. 
     * Packets move between endpoints
     * @returns The new poses of the many packets between the two endpoints
     */
    std::vector<geometry_msgs::msg::Point> getPacketPoints(
        double packet_dist);

    double packet_size_;

    std::map<std::string, LinkInformation> link_infos_;

    // The position of the two ends
    geometry_msgs::msg::Point point_i_;
    geometry_msgs::msg::Point point_o_;
    bool valid_point_i_;
    bool valid_point_o_;

    // Used in shifting the packets;
    // the more the num_steps_ the smoother the packets motions 
    // (though depends on 'counter_update_rate' too)
    const int num_steps_{ 128 };
    // changes from 0 to 'num_steps_ - 1' to simulate the packets displacement
    int counter_;
    int counter_update_rate_;
    
    // The maximum distance up to which the rsu-obu pair remain connected
    double max_dist_;

    // Need to keep base_frame for later use
    std::string frame_id_;
};

/**
 * @brief Represents an indirect connection where there is one connection between
 * the RSU and the cloud, and another between the cloud and the vehicle
 */
struct LinkPair
{
Link rsu2cloud;
Link cloud2vehicle;
};


/**
 * @brief Represents a bounding box
 */
class BoundingBox {
public:

    /**
     * @brief Constructor
     * @param position The position of the bounding box
     * @param yaw The heading of the bounding box
     * @param dimension The dimension of the bounding box
     * @param object_class Either of "UNKNOWN, VEHICLE, PERSON, ANIMAL, OTHER"
     */
    BoundingBox(
        const double position[3]
        , const double yaw
        , const double dimension[3]
        , std::string object_class);
    
    /**
     * @brief Deconstructor
    */
    ~BoundingBox();

private:
    // Give the Detection class direct access to the member variables of this class
    friend class Detection;
    
    /**
     * @brief Constructs and initialises the bounding box
     * @param position
     * @param yaw
     * @param dimension
    */
    void initialise(
        const double position[3]
        , const double yaw
        , const double dimension[3]);
    
    // The general pose of this box (refers to the centre of the cuboid)
    Eigen::Isometry3d pose_;
    
    double dimension_[3];

    // The class of this detected object. Can be either of "UNKNOWN, VEHICLE, PERSON, ANIMAL, OTHER"
    std::string object_class_;
    
    // Used to calculate and update the opacity of the bounding box
    // (to provide gradual fading of the box over time)
    std::chrono::time_point<std::chrono::steady_clock> creation_time_;
};


/**
 * @brief Represents a detection franchise as a set of bounding boxes
*/
class Detection : public Entity {
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param base_frame The reference frame used for visualisations, usually 'world' or 'map'
     * @param id The name of the detection
     * @param colour The colour of the detection
     * @param fade_time Determines if and when to fully fade the bounding boxes
     */
    Detection(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , const std::string& id
        , const std::string& colour
        , int fade_time = 0);

    /**
     * @brief Deconstructor
    */
    ~Detection();

    /**
     * @brief Removes all the bounding boxes stored so far
    */
    void deleteDetections();
    
    /**
     * @brief Adds a series of new detection to the group of detections
     * @param object_infos A list of object info messages to get the detection instances from
    */
    void addDetections(
        const std::vector<dm_object_info_msgs::msg::ObjectInfo>& object_infos);
    
    /**
     * @brief Publish the updates to RViz
    */
    void publishUpdates();

    /**
     * @brief Sets the base_altitude member attribute
     * @param base_altitude
    */
    void setBaseAltitude(double base_altitude);
    
private:
    /**
     * @brief Get the class that has the highest probability among all
     * @param classes The classes of an object info message
     * @returns The index of the class
    */
    std::size_t getMaxClass(
        const std::vector<dm_object_info_msgs::msg::ObjectClass>& classes);

    // The bounding boxes of the detection
    std::vector<BoundingBox> bboxes_;

    // The colour of the bounding boxes
    rviz_visual_tools::Colors colour_;
    
    // The time of fading
    int fade_time_;

    // The opacity of the fill colour of the bounding boxes
    double fill_opacity_;
    
	// Base altitude for cases where the incoming DM messages
    // do not contain altitude data
    double base_altitude_;
};


/**
 * @brief Represents a freespace segment
 */
class Segment {
public:

    /**
     * @brief Constructor
     * @param begin_position The position of the start of the segment
     * @param end_position The position of the end of the segment
     * @param freespace_width The width of the segment
     */
    Segment(
        const double begin_position[3]
        , const double end_position[3]
        , double freespace_width);
    
    /**
     * @brief Deconstructor
    */
    ~Segment();

private:
    // Give the Freespace class direct access to the member variables of this class
    friend class Freespace;
    
    /**
     * @brief Constructs and initialises the segment
     * @param begin_position The position of the start of the segment
     * @param end_position The position of the end of the segment
     * @param freespace_width
    */
    void initialise(
        const double begin_position[3]
        , const double end_position[3]
        , double freespace_width);

    std::vector<geometry_msgs::msg::Point> points_;
};


/**
 * @brief Represents a freespace area
*/
class Freespace : public Entity {
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param base_frame The reference frame used for visualisations, usually 'world' or 'map'
     * @param id The id of the freespace
     * @param freespace_colour Freespace's colour
     * @param freespace_width Freespace's width
     */
    Freespace(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , const std::string& id
        , const std::string& freespace_colour
        , double freespace_width);

    /**
     * @brief Deconstructor
    */
    ~Freespace();

    /**
     * @brief Removes all the segment stored so far
    */
    void deleteFreespaces();
    
    /**
     * @brief Adds a series of new detection to the group of detections
     * @param freespace_infos A list of freespace info messages to get the segment instances from
    */
    void addFreespaces(
        const std::vector<dm_freespace_info_msgs::msg::FreespaceInfo>& freespace_infos);
    
    /**
     * @brief Publish the updates to RViz
    */
    void publishUpdates();
    
    /**
     * @brief Sets the base_altitude member attribute
     * @param base_altitude
    */
    void setBaseAltitude(double base_altitude);

private:
    rviz_visual_tools::Colors freespace_colour_;

    double freespace_width_;
    
    std::vector<Segment> segments_;
    
	// Base altitude for cases where the incoming DM messages
    // do not contain altitude data
    double base_altitude_;

    visualization_msgs::msg::Marker freespaces_;
};

} // namespace entities

#endif // ENTITIES_HPP_