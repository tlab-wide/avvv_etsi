/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Provides tools to read point cloud data from .pcd files and publish
 * them on a topic to display on Rviz
*/

#ifndef VISUALLY__POINTCLOUD_TOOLS_HPP_
#define VISUALLY__POINTCLOUD_TOOLS_HPP_

// C++
#include <string>
#include <vector>
#include <chrono>
#include <ratio>

// CMake
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


namespace pointcloud_tools
{

/**
 * @brief The chief class prividing tools to display pointcloud files
 * @note pointclouds are published as a PointCloud2 message on the topic "/pointcloud2"
 */
class PointcloudTools {
public:
    /**
     * @brief Constructor
     * @param node A ROS node
     * @param base_frame The reference frame used for visualisations, usually
     * 'world' or 'map'
     */
    PointcloudTools(
        rclcpp::Node& node
        , const std::string& base_frame);

    /**
     * @brief Deconstructor
     */
    ~PointcloudTools();

    /**
     * @brief The chief method to load pointcloud from file and publish it
     * @param file_path The path to the file e.g. 'pointcloud.pcd'
     * @param offset An arbitrary offset to shift the pointcloud points
     * @param centre When true, all points of the pointcloud with geographically
     * accurate positions get shifted towards the origin point (0, 0, 0)
     */
    void loadFile(
        const std::string& file_path
        , const double offset[3]
        , bool centre = false);

private:
    /**
     * @brief Waits for a subscriber to be created on the publishing topic of this class
     * @note This method keeps blocking the process until a related subscriber turns up
     */
    void waitForSubscriber();
    
    /**
     * @brief Displaces all points in the given cloud to gather them around the origin
     * @param cloud
     */
    void centreCloud(pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    /**
     * @brief Displaces all points in the given cloud to gather them around the origin
     * @param cloud
     * @param offset
     */
    void applyOffset(
        pcl::PointCloud<pcl::PointXYZ>& cloud
        , const double offset[3]);
    
    /**
     * @brief Converts the given value to bytes and appends them to the given data
     * @param value
     * @param data
     */
    void copyValueToData(float value, std::vector<unsigned char>& data);
    
    /**
     * @brief Creates a ROS PointCloud2 message out ot the given PCL cloud
     * @param cloud
     * @param pc2_msg The ROS message corresponding to the given cloud
     */
    void createPointcloud2Msg(
        const pcl::PointCloud<pcl::PointXYZ>& cloud
        , sensor_msgs::msg::PointCloud2& pc2_msg);

    // ROS node to handle ROS specific tasks such as publishing, logging, etc 
    rclcpp::Node& node_;
    
    // The visualisation reference frame
    std::string base_frame_;
    
    // The publisher to publish pointcloud data
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

};

} // namespace pointcloud_tools

#endif // VISUALLY__POINTCLOUD_TOOLS_HPP_