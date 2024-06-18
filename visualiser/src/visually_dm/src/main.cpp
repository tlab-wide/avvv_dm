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


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node{ std::make_shared<rclcpp::Node>("simulator") };

	// Declaring parameters
	node->declare_parameter<double>("base_altitude");
	node->declare_parameter<std::string>("pcd_file");
	node->declare_parameter<std::string>("rsu_detection_topic");
	node->declare_parameter<std::string>("obu_detection_topic");
	node->declare_parameter<std::string>("freespace_topic");
	node->declare_parameter<std::string>("signal_topic");
	node->declare_parameter<std::vector<std::string>>("rsu_topics");
	node->declare_parameter<std::vector<std::string>>("obu_topics");
	node->declare_parameter<std::string>("rsu_detected_colour");
	node->declare_parameter<std::string>("obu_detected_colour");
	node->declare_parameter<std::string>("freespace_colour");
	node->declare_parameter<double>("freespace_width");
	node->declare_parameter<std::vector<std::string>>("link_list");
	node->declare_parameter<std::vector<std::string>>("rsu_list");
	node->declare_parameter<std::vector<std::string>>("obu_list");
	node->declare_parameter<std::vector<std::string>>("cloud_list");
	node->declare_parameter<std::vector<std::string>>("signal_list");
	node->declare_parameter<double>("rsu_obu_con_dist");
	node->declare_parameter<std::vector<double>>("pointcloud_offset");

	//////////////////////////////////////////////////////////////////////////
	///////////////////////////// Configurations /////////////////////////////

	const std::string base_frame = "map";

	// Base altitude for cases where the incoming DM messages do not contain altitude
	// data
	double base_altitude{ node->get_parameter("base_altitude").as_double() };

	// The address of the pointcloud file
	std::string pcd_filename{ node->get_parameter("pcd_file").as_string() };

	// The topics in ROSBAG files corresponding to different concepts
	std::string rsu_detection_topic{ node->get_parameter("rsu_detection_topic").as_string() };
	std::string obu_detection_topic{ node->get_parameter("obu_detection_topic").as_string() };
	std::string freespace_topic{ node->get_parameter("freespace_topic").as_string() };
	std::string signal_topic{ node->get_parameter("signal_topic").as_string() };
	std::vector<std::string> rsu_topics{ node->get_parameter("rsu_topics").as_string_array() };
	std::vector<std::string> obu_topics{ node->get_parameter("obu_topics").as_string_array() };

	// The different detection colours
	std::string rsu_detected_colour{ node->get_parameter("rsu_detected_colour").as_string() };
	std::string obu_detected_colour{ node->get_parameter("obu_detected_colour").as_string() };
    std::string freespace_colour{ node->get_parameter("freespace_colour").as_string() };

	// The width of the freespace map
	double freespace_width{ node->get_parameter("freespace_width").as_double() };

	// The link endpoints
	std::vector<std::string> link_list{ node->get_parameter("link_list").as_string_array() };

	// The ID and pose of the RSU, OBU and signal light
	std::vector<std::string> rsu_list{ node->get_parameter("rsu_list").as_string_array() };

	std::vector<std::string> obu_list{ node->get_parameter("obu_list").as_string_array() };

	std::vector<std::string> cloud_list{ node->get_parameter("cloud_list").as_string_array() };
	
	std::vector<std::string> signal_list{ node->get_parameter("signal_list").as_string_array() };

	// The maximum connection range between the RSU and the OBU
	const double rsu_obu_con_dist{ node->get_parameter("rsu_obu_con_dist").as_double() };
	
	// The offset of the pointcloud data
	std::vector<double> pointcloud_offset_vector{ node->get_parameter("pointcloud_offset").as_double_array() };
	double pointcloud_offset[3]{
		pointcloud_offset_vector[0]
		, pointcloud_offset_vector[1]
		, pointcloud_offset_vector[2]};

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////

	// Tools

	manager::Visualiser visualiser(
		node
		, base_frame
		, rsu_obu_con_dist);

	visualiser.setBaseAltitude(base_altitude);

	pointcloud_tools::PointcloudTools apt(*node, base_frame);

	try {
		visualiser.addDetection(
			rsu_detection_topic
			, rsu_detected_colour);

		visualiser.addDetection(
			obu_detection_topic
			, obu_detected_colour);

		visualiser.addFreespace(freespace_topic, freespace_colour, freespace_width);

		visualiser.addSignalList(signal_list, signal_topic);

		visualiser.addLinkList(link_list);

		visualiser.addRsuList(rsu_list, rsu_topics);

		visualiser.addObuList(obu_list, obu_topics);

		visualiser.addCloudList(cloud_list);

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
