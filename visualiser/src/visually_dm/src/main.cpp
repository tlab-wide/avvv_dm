/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Entry point for package visually_dm
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
#include <visually_dm/rviz_tools.hpp>
#include <visually_dm/pointcloud_tools.hpp>
#include <visually_dm/manager.hpp>
#include <visually_dm/net_status.hpp>


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node{ std::make_shared<rclcpp::Node>("simulator") };

	// Declaring parameters
	node->declare_parameter<double>("base_altitude");
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
	node->declare_parameter<std::vector<std::string>>("rsu_detection_topics");
	node->declare_parameter<std::vector<std::string>>("obu_detection_topics");
	node->declare_parameter<std::vector<std::string>>("rsu_freespace_topics");
	node->declare_parameter<std::vector<std::string>>("obu_freespace_topics");
	node->declare_parameter<std::vector<std::string>>("rsu_signal_topics");
	node->declare_parameter<std::vector<std::string>>("obu_signal_topics");
	node->declare_parameter<std::vector<std::string>>("obu_topics");
	node->declare_parameter<std::string>("target_rsu_id");
	node->declare_parameter<std::string>("target_obu_id");
	node->declare_parameter<std::string>("rsu_detected_colour");
	node->declare_parameter<std::string>("obu_detected_colour");
	node->declare_parameter<std::string>("rsu_freespace_colour");
	node->declare_parameter<std::string>("obu_freespace_colour");
	node->declare_parameter<double>("freespace_width");
	node->declare_parameter<std::vector<std::string>>("link_topics");
	node->declare_parameter<std::vector<std::string>>("rsu_list");
	node->declare_parameter<std::vector<std::string>>("obu_list");
	node->declare_parameter<std::vector<std::string>>("cloud_list");
	node->declare_parameter<std::vector<std::string>>("signal_list");
	node->declare_parameter<double>("rsu_obu_con_dist");
	node->declare_parameter<std::string>("off_hm_path");
	node->declare_parameter<int>("off_hm_attr");
	node->declare_parameter<bool>("on_hm");

	//////////////////////////////////////////////////////////////////////////
	///////////////////////////// Configurations /////////////////////////////

	const std::string base_frame = "map";

	// Base altitude for cases where the incoming DM messages do not contain altitude
	// data
	double base_altitude{ node->get_parameter("base_altitude").as_double() };

	// The address of the pointcloud file
	std::string pcd_filename{ node->get_parameter("pcd_file").as_string() };
	std::vector<double> map_offset_vector{ node->get_parameter("map_offset").as_double_array() };

	// The topics in ROSBAG files corresponding to different concepts
	std::vector<std::string> rsu_detection_topics{ node->get_parameter("rsu_detection_topics").as_string_array() };
	std::vector<std::string> obu_detection_topics{ node->get_parameter("obu_detection_topics").as_string_array() };
	std::vector<std::string> rsu_freespace_topics{ node->get_parameter("rsu_freespace_topics").as_string_array() };
	std::vector<std::string> obu_freespace_topics{ node->get_parameter("obu_freespace_topics").as_string_array() };
	std::vector<std::string> rsu_signal_topics{ node->get_parameter("rsu_signal_topics").as_string_array() };
	std::vector<std::string> obu_signal_topics{ node->get_parameter("obu_signal_topics").as_string_array() };
	std::vector<std::string> obu_topics{ node->get_parameter("obu_topics").as_string_array() };

	// The different detection colours
	std::string rsu_detected_colour{ node->get_parameter("rsu_detected_colour").as_string() };
	std::string obu_detected_colour{ node->get_parameter("obu_detected_colour").as_string() };
    std::string rsu_freespace_colour{ node->get_parameter("rsu_freespace_colour").as_string() };
    std::string obu_freespace_colour{ node->get_parameter("obu_freespace_colour").as_string() };

	// The width of the freespace map
	double freespace_width{ node->get_parameter("freespace_width").as_double() };

	// The link endpoints
	std::vector<std::string> link_topics{ node->get_parameter("link_topics").as_string_array() };

	// The ID and pose of the RSU, OBU and signal light
	std::vector<std::string> rsu_list{ node->get_parameter("rsu_list").as_string_array() };

	std::vector<std::string> obu_list{ node->get_parameter("obu_list").as_string_array() };

	std::vector<std::string> cloud_list{ node->get_parameter("cloud_list").as_string_array() };
	
	std::vector<std::string> signal_list{ node->get_parameter("signal_list").as_string_array() };

	// Target RSU and OBU IDs
	std::string target_rsu_id{ node->get_parameter("target_rsu_id").as_string() };
	std::string target_obu_id{ node->get_parameter("target_obu_id").as_string() };

	// The maximum connection range between the RSU and the OBU
	const double rsu_obu_con_dist{ node->get_parameter("rsu_obu_con_dist").as_double() };

	// Offline heatmap file path
	std::string off_hm_path{ node->get_parameter("off_hm_path").as_string() };

	int off_hm_net_attr{ static_cast<int>(node->get_parameter("off_hm_attr").as_int()) };

	bool on_hm{ node->get_parameter("on_hm").as_bool() };
	
	// Network attributes
	net_status::NetworkField link_colour{
		net_status::intToNetworkField(node->get_parameter("link_colour").as_int()) };
	net_status::NetworkField link_thickness{
		net_status::intToNetworkField(node->get_parameter("link_thickness").as_int()) };
	net_status::NetworkField link_packet_density{
		net_status::intToNetworkField(node->get_parameter("link_packet_density").as_int()) };
	net_status::NetworkField link_opacity{
		net_status::intToNetworkField(node->get_parameter("link_opacity").as_int()) };

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
	
	double map_offset[3]{ map_offset_vector[0] , map_offset_vector[1], map_offset_vector[2] };

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////

	// Tools

	manager::Visualiser visualiser(
		node,
		base_frame,
		link_colour,
		link_thickness,
		link_packet_density,
		link_opacity,
		net_param_ranges,
		rsu_obu_con_dist);

	visualiser.setBaseAltitude(base_altitude);

	pointcloud_tools::PointcloudTools apt(*node, base_frame);

	try {
		for (auto& rsu_detection_topic : rsu_detection_topics)
			visualiser.addRsuDetection(
				rsu_detection_topic,
				rsu_detected_colour);

		for (auto& obu_detection_topic : obu_detection_topics)
			visualiser.addObuDetection(
				obu_detection_topic,
				obu_detected_colour,
				on_hm);

		for (auto& rsu_freespace_topic : rsu_freespaces_topics)
			visualiser.addRsuFreespace(
				rsu_freespace_topic,
				rsu_freespace_colour,
				freespace_width);

		for (auto& obu_freespace_topic : obu_freespaces_topics)
			visualiser.addRsuFreespace(
				obu_freespace_topic,
				obu_freespace_colour,
				freespace_width,
				on_hm);

		
		visualiser.addSignalList(
			signal_list,
			obu_signal_topics,
			on_hm);

		visualiser.addRsuList(rsu_list, rsu_topics);

		visualiser.addObuList(obu_list, obu_topics);

		visualiser.addCloudList(cloud_list);
		
		visualiser.addLinkList(link_topics);

		if (off_hm_path.length())
			visualiser.addOfflineHeatmap(
				off_hm_path,
				target_rsu_id,
				target_obu_id,
				off_hm_net_attr);
		
		// Loading the Pointcloud file
		if (pcd_filename.length())
			apt.loadFile(pcd_filename, map_offset, false);

		rclcpp::spin(node);
	}
	catch (std::runtime_error& e) {
		RCLCPP_ERROR(node->get_logger(), e.what());
	}

	// Shutting down ROS
	rclcpp::shutdown();
	
	return 0;
}
