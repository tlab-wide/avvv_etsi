/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Entry point for package visually
*/


// C++
#include <memory>
#include <vector>
#include <utility>
#include <algorithm>
#include <cstdlib>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <builtin_interfaces/msg/time.hpp>

// Package
#include <cpm_ros_msgs/msg/cpm_message.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <visually/rviz_tools.hpp>
#include <visually/pointcloud_tools.hpp>
#include <visually/rosbag.hpp>
#include <visually/manager.hpp>
#include <visually/net_status.hpp>


using namespace std::chrono_literals;


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node{ std::make_shared<rclcpp::Node>("simulator") };

	// Declaring parameters
	node->declare_parameter<std::string>("pcd_file");
	node->declare_parameter<std::vector<double>>("map_offset");
	node->declare_parameter<int>("link_colour");
	node->declare_parameter<int>("link_thickness");
	node->declare_parameter<int>("link_packet_density");
	node->declare_parameter<int>("link_opacity");
	node->declare_parameter<double>("delay_best");
	node->declare_parameter<double>("delay_worst");
	node->declare_parameter<double>("jitter_best");
	node->declare_parameter<double>("jitter_worst");
	node->declare_parameter<int>("rssi_best");
	node->declare_parameter<int>("rssi_worst");
	node->declare_parameter<double>("packet_loss_best");
	node->declare_parameter<double>("packet_loss_worst");
	node->declare_parameter<std::vector<std::string>>("topics");
	node->declare_parameter<std::string>("target_rsu_id");
	node->declare_parameter<std::string>("target_obu_id");
	node->declare_parameter<double>("rsu_obu_con_dist");
	node->declare_parameter<std::string>("off_hm_path");
	node->declare_parameter<int>("off_hm_attr");
	node->declare_parameter<bool>("on_hm");

	//////////////////////////////////////////////////////////////////////////
	///////////////////////////// Configurations /////////////////////////////

	const std::string base_frame = "map";

	// Derived from the parameter centre
	// Topics
	std::vector<std::string> topics{ node->get_parameter("topics").as_string_array() };

	// The address of the pointcloud file
	std::string pcd_filename{ node->get_parameter("pcd_file").as_string() };
	std::vector<double> map_offset{ node->get_parameter("map_offset").as_double_array() };

	// Network attributes
	net_status::NetworkField link_colour{
		net_status::intToNetworkField(node->get_parameter("link_colour").as_int()) };
	net_status::NetworkField link_thickness{
		net_status::intToNetworkField(node->get_parameter("link_thickness").as_int()) };
	net_status::NetworkField link_packet_density{
		net_status::intToNetworkField(node->get_parameter("link_packet_density").as_int()) };
	net_status::NetworkField link_opacity{
		net_status::intToNetworkField(node->get_parameter("link_opacity").as_int()) };

	// Target RSU and OBU IDs
	std::string target_rsu_id{ node->get_parameter("target_rsu_id").as_string() };
	std::string target_obu_id{ node->get_parameter("target_obu_id").as_string() };
	
	const double rsu_obu_con_dist{ node->get_parameter("rsu_obu_con_dist").as_double() };

	// Offline heatmap file path
	std::string off_hm_path{ node->get_parameter("off_hm_path").as_string() };

	int off_hm_net_attr{ static_cast<int>(node->get_parameter("off_hm_attr").as_int()) };

	bool on_hm{ node->get_parameter("on_hm").as_bool() };
	
	double pointcloud_offset[3]{ map_offset[0], map_offset[1], map_offset[2] };

	net_status::NetParamRanges net_param_ranges{
		node->get_parameter("delay_best").as_double()
		, node->get_parameter("delay_worst").as_double()
		, node->get_parameter("jitter_best").as_double()
		, node->get_parameter("jitter_worst").as_double()
		, node->get_parameter("rssi_best").as_int()
		, node->get_parameter("rssi_worst").as_int()
		, node->get_parameter("packet_loss_best").as_double()
		, node->get_parameter("packet_loss_worst").as_double()
	};

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////

	// Tools

	manager::Visualiser visualiser(
		node
		, base_frame
		, link_colour
		, link_thickness
		, link_packet_density
		, link_opacity
		, net_param_ranges
		, rsu_obu_con_dist);

	pointcloud_tools::PointcloudTools apt(*node, base_frame);

	try {
		// Reading and processing all available topics in the ROSBAG file
		for (const auto& topic : topics)
			visualiser.analyseTopic(topic, on_hm);

		if (off_hm_path.length())
			visualiser.addOfflineHeatmap(off_hm_path, target_rsu_id, target_obu_id, off_hm_net_attr);
			
		// Loading the Pointcloud file
		if (pcd_filename.length())
			apt.loadFile(pcd_filename, pointcloud_offset, false);

		rclcpp::spin(node);
	}
	catch (std::runtime_error& e) {
		RCLCPP_ERROR(node->get_logger(), e.what());
	}

	// Shutting down ROS
	rclcpp::shutdown();
	
	return 0;
}
