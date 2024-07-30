/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Provides tools to visualise offline and real time heatmap
*/

#ifndef VISUALLY__HEATMAP_HPP_
#define VISUALLY__HEATMAP_HPP_

// C++
#include <vector>
#include <string>
#include <limits>
#include <regex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// CSV2
#include <csv2/reader.hpp>

// rviz_visual_tools
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// Package
#include <visually/entities.hpp>
#include <cpm_ros_msgs/msg/cpmn.hpp>
#include <visually/net_status.hpp>


namespace heatmap
{

/**
 * @brief Represents one Heatmap point in space
*/
struct HeatmapPoint
{
    // The colour associated to the point
    rviz_visual_tools::Colors colour{};
    // The position of the point
    geometry_msgs::msg::Point geo_point{};
};


/**
 * @brief Represents an offline/online heatmap as a marker publishing entity
*/
class HeatmapRecorder: public entities::Entity
{
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param id The ID of the entity
     * @param base_frame The base frame to publish everything in (usually 'map')
     * @param net_status_repr The network status representation parameters
     * @param network_attr The network attribute the heatmap represents
     * @param opacity The opacity of the markers of the heatmap
     * @param scale The scale of the markers of the heatmap
    */
    HeatmapRecorder(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& id
        , const std::string& base_frame
        , const net_status::NetStatusRepr& net_status_repr
        , int network_attr
        , double opacity
        , double scale);

    /**
     * @brief Deconstructor
    */
    ~HeatmapRecorder();

    /**
     * @returns The list of heatmap points
    */
    const std::vector<HeatmapPoint>& getHeatmapPoints() const;

    /**
     * @brief Publishes the updates to RViz
    */
    virtual void publishUpdates();

protected:
    /**
     * @brief Adds a new point to the list of heatmap points
     * @param x The x position of the new heatmap point
     * @param y The y position of the new heatmap point
     * @param z The z position of the new heatmap point
     * @param colour The colour of the new heatmap point
    */
    void addNewHeatmapPoint(
        double x
        , double y
        , double z
        , rviz_visual_tools::Colors colour);

    std::vector<HeatmapPoint> heatmap_points_;

    // The network status attribute parameters including the range of the network attributes
    net_status::NetStatusRepr net_status_repr_;

    // The attribute of the network this heatmap represents
    int network_attr_;
    
    // The opacity at which the heatmap points appear in RViz
    double opacity_;

    // The scale of the markers of the heatmap
    double scale_;

    // The ID of the markers
    std::size_t marker_id_;
};

/**
 * @brief Tool to place an offline heatmap in the RViz simulator field
*/
class OfflineHeatmap: public HeatmapRecorder
{
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param id The ID of the entity
     * @param base_frame The base frame to publish everything in (usually 'map')
     * @param net_status_repr The network status representation parameters
     * @param network_attr The network attribute the heatmap represents
    */
    OfflineHeatmap(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& id
        , const std::string& base_frame
        , const net_status::NetStatusRepr& net_status_repr
        , int network_attr);

    /**
     * @brief Deconstructor
    */
    ~OfflineHeatmap();

    /**
     * @brief Reads the heatmap points from file
     * @param file_path
    */
    void withdrawHeatmapPoints(const std::string& file_path);

    /**
     * @brief Creates the visualisation markers out of the gathered heatmap points
    */
    void buildMarkers();

    /**
     * @brief Publishes the updates to RViz
    */
    void publishUpdates();

private:
    visualization_msgs::msg::Marker marker_;

}; // class OfflineHeatmap


class OnlineHeatmap: public HeatmapRecorder
{
public:
    /**
     * @brief Constructor
     * @param node A ROS node to use to initialise publishers
     * @param id The ID of the entity
     * @param base_frame The base frame to publish everything in (usually 'map')
     * @param net_status_repr The network status representation parameters
     * @param network_attr The network attribute the heatmap represents
    */
    OnlineHeatmap(
        const std::shared_ptr<rclcpp::Node>& node
        , const std::string& id
        , const std::string& base_frame
        , const net_status::NetStatusRepr& net_status_repr
        , int network_attr);

    /**
     * @brief Deconstructor
    */
    ~OnlineHeatmap();

    /**
     * @brief Adds a new online heatmap point to the list of points
     * @param x The x position of the new heatmap point
     * @param y The y position of the new heatmap point
     * @param z The z position of the new heatmap point
     * @param value The value of the new heatmap point
    */
    void addNewOnlineHeatmapPoint(
        double x
        , double y
        , double z
        , double value);

    /**
     * @brief Publishes the updates to RViz
    */
    void publishUpdates();

private:
    // The marker that handles the visualisation of the online heatmap
    visualization_msgs::msg::Marker line_strip_;

    // The ID of the current marker to publish
    std::size_t marker_id_;
    // Used to keep track of how many of the points have 
    // published so far and how many to publish
    std::size_t count_;
}; // class OnlineHeatmap

} // namespace heatmap

#endif // VISUALLY__HEATMAP_HPP_