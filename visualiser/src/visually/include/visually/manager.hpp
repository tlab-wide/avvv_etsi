/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Includes the common constants and functions used in AVNV
*/

#ifndef VISUALLY__MANAGER_HPP_
#define VISUALLY__MANAGER_HPP_

// C++
#include <memory>
#include <vector>
#include <utility>
#include <mutex>
#include <chrono>
#include <regex>
#include <set>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Package
#include <visually/rviz_tools.hpp>
#include <visually/transforms.hpp>
#include <cpm_ros_msgs/msg/cpm_message.hpp>
#include <cpm_ros_msgs/msg/cpmn.hpp>
#include <cpm_ros_msgs/msg/network_status.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <visually/heatmap.hpp>
#include <visually/net_status.hpp>


namespace manager
{

// Configurations
const std::string rsu_colour_{ "white" }; // Colour of the RSUs
const std::string obu_colour_{ "white" }; // Colour of the OBUs
const std::string rsu_detected_colour_{ "white" }; // Colour of the RSU-detected vehicles
const std::string obu_detected_colour_{ "white" }; // Colour of the OBU-detected vehicles
const std::string obu_received_colour_{ "red" }; // Colour of the detected vehicles the OBUs receive
const std::string rsu_sent_vehicle_colour_{ "green" }; // Colour of the detected vehicles that RSUs send

// The scales to which the CPM fields are.
constexpr double distance_scale_{ 0.01 };
constexpr double speed_scale_{ 0.01 };
constexpr double angle_scale_{ 0.1 * M_PI / 180.0};
constexpr double dimension_scale_{ 0.1 };
constexpr unsigned delay_scale_{ 10'000 };
constexpr double geo_scale_{ 1.0e-7 };


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
     * @param link_colour Parameter of network status to represent
     * @param link_thickness Parameter of network status to represent
     * @param link_packet_density Parameter of network status to represent
     * @param link_opacity Parameter of network status to represent
     * @param ranges The network parameters upper and lower bounds
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
     * @brief Categorises the given topic. Then registers the RSU and OBU IDs
     * into the Visualiser system. Finally for each topic, creates an
     * rclcpp::Subscription with regards to the message type.
     * @param topic
     * @param add_online_heatmap Creates an additional real time heatmap visual type
     * along with the main type if the topic is related to CPMN
    */
    void analyseTopic(
        const std::string& topic
        , bool add_online_heatmap);

    /**
     * @brief Adds offline an heatmap type to art.
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
     * @brief Searches the classification vector for the label with the maximum probability
     * @param classifications
    */
    uint8_t maxLabel(
        const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification>& classifications);

    /**
     * @brief The callback related to the RSU CPM. Displays the transmitted detected
     * objects as bounding boxes
     * @param msg
     * @param det_id The ID of the detection type in art
    */
    void cpmDetectionMessageCallback(
        const cpm_ros_msgs::msg::CPMMessage& msg
        , const std::string& det_id);

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
     * @brief Displays the detected objects of an RSU by bounding boxes
     * @param msg
     * @param det_id
    */
    void detectionMessageCallback(
        const autoware_auto_perception_msgs::msg::PredictedObjects& msg
        , const std::string& det_id);

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
     * @brief Updates the network status between the OBU and RSU determined by the given ID
     * @param msg
     * @param id Determines the RSU and OBU involved
     * @param obu_id The OBU ID to update the echo lines colour
     * @param obu_id The RSU ID involved in the RSU-OBU link
    */
    void networkStatusMessageCallback(
        const cpm_ros_msgs::msg::NetworkStatus& msg
        , const std::string& id
        , const std::string& obu_id
        , const std::string& rsu_id);

    /**
     * @brief Updates the online heatmap of the OBU-RSU pair with the given ID
     * @param msg
     * @param id
    */    
    void onlineHeatmapCallback(
        const cpm_ros_msgs::msg::CPMN& msg
        , const std::string& id);
    
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
    std::vector<std::shared_ptr<rclcpp::Subscription<cpm_ros_msgs::msg::CPMMessage>>> cpm_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<cpm_ros_msgs::msg::CPMN>>> cpmn_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<cpm_ros_msgs::msg::CPMN>>> online_heatmap_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<cpm_ros_msgs::msg::NetworkStatus>>> netsta_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>>> tf_subscriptions_;
    std::vector<std::shared_ptr<rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>>> detobj_subscriptions_;
    
    net_status::NetStatusRepr net_status_repr_;

    double rsu_obu_con_dist_;
};

} // namespace common

#endif // VISUALLY__MANAGER_HPP_