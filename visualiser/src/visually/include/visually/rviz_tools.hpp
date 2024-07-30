/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Provides tools to simulate the entities and processes of an autonomous
 * vehicle network
*/

#ifndef VISUALLY__RVIZ_TOOLS_HPP_
#define VISUALLY__RVIZ_TOOLS_HPP_

// C++
#include <cmath>
#include <vector>
#include <string>
#include <functional>
#include <queue>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

// Rviz Visual Tools
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// Package
#include <visually/net_status.hpp>
#include <visually/entities.hpp>
#include <visually/heatmap.hpp>


namespace rviz_tools
{

using namespace std::chrono_literals;

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
     * @brief Updates the pose of a vehicle with the given ID based on the
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
     * @brief Updates the pose of an RSU with the given ID based on the
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
     * @brief Adds a new RSU-vehicle link to the network
     * @param id
     * @param max_dist The maximum distance at which the link stays visually connected
     * @note The given ID must be unique, or this will be ignored
     */
    void addLink(const std::string& id, double max_dist);

    /**
     * @brief Updates the features of the link corresponding to the given ID
     * @param id
     * @param colour
     * @param line_thickness
     * @param opacity
     * @param packet_dist
     * @note If the given ID is invalid, this will be ignored
     */
    void updateLinkSpec(
        const std::string& id
        , rviz_visual_tools::Colors colour
        , double line_thickness
        , double opacity
        , double packet_dist);

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
     * @brief Adds a single detection instance to the detection
     * @param id
     * @param pose The pose of the new instance
     * @param dimension The dimension of the new instance
     * @note If the given ID is invalid, this will be ignored
     */
    void updateDetection(
        const std::string& id
        , const geometry_msgs::msg::Pose& pose
        , const geometry_msgs::msg::Vector3& dimension);

    /**
     * @brief Adds several detection instances to the detection
     * @param id
     * @param poses The pose of the new instances
     * @param dimensions The dimension of the new instances
     * @note If the given ID is invalid, this will be ignored
     */
    void updateDetection(
        const std::string& id
        , const std::vector<geometry_msgs::msg::Pose>& poses
        , const std::vector<geometry_msgs::msg::Vector3>& dimensions);
    
    /**
     * @brief Adds several detection instances to the detection
     * @param id
     * @param positions The x-y-z position of the new instances
     * @param yaws The heading of the new instances
     * @param dimension The dimension of the new instances
     * @note If the given ID is invalid, this will be ignored
     */
    void updateDetection(
        const std::string& id
        , const std::vector<double*>& positions
        , const std::vector<double>& yaws
        , const std::vector<double*>& dimensions);

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
        , double x
        , double y
        , double z
        , double value);

    /**
     * @brief Gets the average packet loss of an OBU in connection with several RSUs
     * @param obu_id
    */
    double getAvgObuRsuPacketLoss(const std::string& obu_id);

    /**
     * @brief Sets the OBU-RSU packet loss value
     * @param obu_id
     * @param rsu_id
     * @param value
    */
    void setObuRsuPacketLoss(
        const std::string& obu_id
        , const std::string& rsu_id
        , double value);

private:
    /**
     * @brief The callback for the timer to display all entities.
     * @note This method is by default called by a timer when an object of this class is created.
     * @note Without calling this method, nothing shows up on Rviz.
     */
    void display();

    void baseDisplay();

    /**
     * @brief Updates all links endpoints position involving the endpoint specified by the give ID
     * @param id The ID of the endpoint
     * @param point The new position of the endpoint
    */
    void updateLinkEndpoints(
        const std::string& id
        , const geometry_msgs::msg::Point& point);

    // ROS node to handle ROS specific tasks such as publishing, logging, etc 
    std::shared_ptr<rclcpp::Node> node_;
    
    // The visualisation reference frame
    std::string base_frame_;
    
    // Maps and vectors of IDs to their corresponding objects    
    std::map<std::string, entities::Mesh> rsus_;
    std::map<std::string, entities::Mesh> obus_;
    std::map<std::string, entities::Mesh> clouds_;

    std::map<std::string, entities::Detection> detections_;
    std::map<std::string, entities::Link> links_;
    std::map<std::string, heatmap::OfflineHeatmap> offline_heatmaps_;
    std::map<std::string, heatmap::OnlineHeatmap> online_heatmaps_;

    std::map<std::string, std::map<std::string, double>> obu_rsu_packet_losses_;

    // The interactive marker callbacks for all the meshes (RSUs, OBUs and clouds)
    std::map<std::string, interactive_markers::InteractiveMarkerServer::FeedbackCallback> callbacks_;

    // Determines the sleep period (indirectly the "updates per second" rate)
    // Note that this value must never be greater that "display_lifetime_base"
    static constexpr std::chrono::milliseconds display_period_{ 100 };
    static constexpr int display_period_base_{ 10 };

    // The timer that updates RViz every 'display_lifetime' milliseconds
    rclcpp::TimerBase::SharedPtr display_timer_;
    rclcpp::TimerBase::SharedPtr base_display_timer_;
    
    // The interactive marker sever for interactive meshes
    interactive_markers::InteractiveMarkerServer int_server_;

    // The echo circle line points
    entities::EchoPoints echo_points_;
};

} // namespace rviz_tools

#endif // VISUALLY__RVIZ_TOOLS_HPP_