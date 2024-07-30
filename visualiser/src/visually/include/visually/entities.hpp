/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Includes the entities used in autonomous vehicle network visualisation
*/

#ifndef VISUALLY__ENTITIES_HPP_
#define VISUALLY__ENTITIES_HPP_

// C++
#include <string>
#include <vector>
#include <utility>
#include <cmath>

// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

// Eigen
#include <Eigen/Dense>

// rviz_visual_tools
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// Package
#include <visually/transforms.hpp>


namespace entities
{

// The number of points that make up the echo circle lines
constexpr std::size_t echo_line_points{ 101 };
// The max radius of the echo circle
constexpr int max_echo_line_radius{ 30 };


/**
 * @brief Distinguishes the types of meshes
*/
enum EntityType
{

vehicle = 0,
rsu,
cloud,

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
 * @brief Base class for every entity in AVVV visual board
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

protected:
    std::string id_;
    
    // The responsible RvizVisualTool
    rviz_visual_tools::RvizVisualTools rvizer_;
};


/**
 * @brief Represents an  entity (vehicle, RSU or cloud) on the Rviz simulator
 */
class Mesh : public Entity {
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param base_frame The reference frame used for visualisations, usually 'world' or 'map'
     * @param entity_type RSU, Vehicle or Cloud
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
     * \note Adjusts the pose of the receiver point of this entity as well
     * \param pose
     */
    void setPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Turns displaying information above the entity on or off
    */
    void toggleDisplayInfo();

    /**
     * @returns Returns the receiver position of the entity
    */
    geometry_msgs::msg::Point getReceiverPoint();

    /**
     * @returns Returns the interactive marker of the entity
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
     * @brief Drafts the publishing of static components of this entity
     * including the static echo lines to RViz and the 3D meshes but does
     * not trigger RViz
     * @param echo_points The full echo points array
    */
    void publishBaseUpdates(
        const EchoPoints& echo_points);

    /**
     * @brief Drafts the publishing of dynamic components of this entity
     * including the moving echo lines and texts to RViz and triggers
     * @param echo_points The full echo points array
    */
    void publishUpdates(const EchoPoints& echo_points);

protected:
    /**
     * @brief Initialises the newly created mesh for the mesh file address and the type
     * @param base_frame
     */
    virtual void initialise(const std::string& base_frame);

    /**
     * @brief Creates the interactive marker associating with this entity
     * @param base_frame The fixed frame of the RViz with respect to which data gets
     * visualised.
     */
    void buildInteractiveMarker(const std::string& base_frame);

    /**
     * @brief Updates the static echo lines (when position changes)
     * @param echo_points The full echo points array
    */
    void updateStaticEchoLines(const EchoPoints& echo_points);

    /**
     * @brief Updates the moving echo line as time goes by
     * @param echo_points The full echo points array
    */
    void updateMovingEchoLine(const EchoPoints& echo_points);

    /**
     * @brief Sets the pose of this entity
     * @param pose
     * @param position
     * @param orientation
     * @note Adjusts the pose of the receiver point of this entity as well
     */
    void setPose(
        const geometry_msgs::msg::Pose& pose
        , const double position[3]
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
    // The size of the text
    geometry_msgs::msg::Vector3 text_scale_;

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
     * @param end_point_id The ID of the entity this link meets at any end
     * @param point The new position of the end point
    */
    void updateEndpoint(
        const std::string& end_point_id
        , const geometry_msgs::msg::Point& point);

    /**
     * @brief Update the specifications of the link
     * @param colour
     * @param thickness
     * @param opacity
     * @param packet_dist The new distance between any two packets
    */
    void updateLinkSpecs(
        rviz_visual_tools::Colors colour
        , double thickness
        , double opacity
        , double packet_dist);

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
    std::vector<geometry_msgs::msg::Point> getPacketPoints();

    // The characteristics of the link
    rviz_visual_tools::Colors colour_;
    double line_thickness_;
    double opacity_;
    double packet_size_;
    double packet_dist_;

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

    visualization_msgs::msg::Marker line_;
    visualization_msgs::msg::Marker spheres_;
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
     */
    BoundingBox(
        const double position[3]
        , const double yaw
        , const double dimension[3]);
    
    /**
     * @brief Constructor
     * @param pose The position and orientation of the bounding box
     * @param dimension The dimension of the bounding box
     */
    BoundingBox(
        const geometry_msgs::msg::Pose& pose
        , const geometry_msgs::msg::Vector3& dimension);

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
     * @param id The ID of the detection
     * @param colour Entity's colour
     * @param fade_time
     */
    Detection(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , const std::string& id
        , const std::string& colour
        , int fade_time = 0);

    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param base_frame The reference frame used for visualisations, usually 'world' or 'map'
     * @param id The ID of the detection
     * @param colour Entity's colour
     * @param fade_time
     */
    Detection(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& base_frame
        , const std::string& id
        , rviz_visual_tools::Colors colour
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
     * @brief Adds a new detection to the group of detections
     * @param pose The pose of the new bounding box
     * @param dimension The dimension of the new bounding box
    */
    void addDetection(
        const geometry_msgs::msg::Pose& pose
        , const geometry_msgs::msg::Vector3& dimension);

    /**
     * @brief Adds a series of new detection to the group of detections
     * @param poses The pose of the new bounding boxes
     * @param dimensions The dimension of the new bounding boxes
    */
    void addDetections(
        const std::vector<geometry_msgs::msg::Pose>& poses
        , const std::vector<geometry_msgs::msg::Vector3>& dimensions);
    
    /**
     * @brief Adds a series of new detection to the group of detections
     * @param positions The position of the new bounding boxes
     * @param yaws The heading of the new bounding boxes
     * @param dimensions The dimension of the new bounding boxes
    */
    void addDetections(
        const std::vector<double*>& positions
        , const std::vector<double>& yaws
        , const std::vector<double*>& dimensions);

    /**
     * @brief Publish the updates to RViz
    */
    void publishUpdates();

private:
    // The bounding boxes of the detection
    std::vector<BoundingBox> bboxes_;

    // The colour of the bounding boxes
    rviz_visual_tools::Colors colour_;
    
    // The time of fading
    int fade_time_;

    // The opacity of the fill colour of the bounding boxes
    double fill_opacity_;
};

} // namespace entities

#endif // VISUALLY__ENTITIES_HPP_