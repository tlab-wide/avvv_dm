"""
This module is for merging DM and ROSBAG files into one ROSBAG file and
creating some reports for the topics, files, and network status
"""
import topics
from nodes import NodesManager
from ros2_interface.ros2file_gen import create_bag_file
from config_avvv import Conf
from network_status import NetworkStatus


# from packet_interface import cpm_interface


def dm_merger() -> None:
    """
    This is the primary function for merging CSV files (dm protocols)
    and ros2 files into one ros2 file with cpm-related topics
    This function also creates reports files for the files, topics, and network status
    :return:
    """

    dm_information: dict = {} # This dictionary keeps information of CSV topics
    ros_dict_information: dict # This dictionary keeps information of ROSBAG topics

    dm_protocols = Conf.dm_protocols

    for dm_protocol in dm_protocols:
        try:
            if dm_protocol == "FreespaceInfo":
                rsu_files, obu_files = Conf.freespace_rsu_files, Conf.freespace_obu_files
                topic_syntax = "freespace_info"
            if dm_protocol == "ObjectInfo":
                rsu_files, obu_files = Conf.object_rsu_files, Conf.object_obu_files
                topic_syntax = "object_info"
            if dm_protocol == "SignalInfo":
                rsu_files, obu_files = Conf.signal_rsu_files, Conf.signal_obu_files
                topic_syntax = "signal_info"

        except Exception as e:
            raise Exception(f"Can't read RSU or OBU files. Error message:{str(e)}")

        if len(rsu_files) == 0 or len(obu_files) == 0:
            raise Exception("RSU or OBU files not found in config file!")

        print(f"Working on {dm_protocol}")

        # Read CSV files and create RSU and OBU objects
        nodes_manager = NodesManager(obu_files, rsu_files)  # TODO Add empty CSV exception (when CSV is empty we get error)

        # Create /RSU_#/[dm_protocol] topics in final ROSBAG
        for rsu in nodes_manager.get_rsu_nodes():
            print(f"Working on RSU: {rsu.get_station_id()}")
            rsu.set_dm_protocol_type(dm_protocol)
            topic = rsu.get_topic_name(topic_syntax)
            rsu_dm_msg_grps = rsu.get_csv_dm_info()
            dm_information[topic] = (dm_protocol, rsu, rsu_dm_msg_grps)

        # Create /OBU_#/RSU_#/[topic_syntax]/network_status and /OBU_#/RSU_#/[topic_syntax]n topics in final ROSBAG
        for obu in nodes_manager.get_obu_nodes():
            obu.set_dm_protocol_type(dm_protocol)
            for rsu in nodes_manager.get_rsu_nodes():
                rsu_station_id = rsu.get_station_id()
                obu_id = obu.get_obu_id()
                sender_cap = rsu.get_csv_rows()
                receiver_cap = obu.get_obu_csv_rows_by_rsu_id(rsu_station_id)

                if len(sender_cap) == 0 or len(receiver_cap) == 0:
                    print(
                        f"""The number of RSU or OBU packets specified in the
                          protocol is zero. File names: {rsu.get_csv_file_name()}
                          , {obu.get_csv_file_name()}""")
                    continue
                
                obu_rsu_network_status = NetworkStatus(rsu_station_id, obu_id, sender_cap, receiver_cap, dm_protocol)

                # Add network status object to RSU object
                rsu.append_network_status(obu_rsu_network_status)

                # Generate /OBU_#/RSU_#/[topic_syntax]/network_status topic
                netstat_topic = obu_rsu_network_status.get_topic(
                    topic_syntax + "/" + Conf.network_status_topic_name_syntax)
                network_status_list = obu_rsu_network_status\
                    .get_network_status_list()
                dm_information[netstat_topic] = network_status_list

                # Generate /OBU_#/RSU_#/[topic_syntax]n topic
                dmn_topic = obu_rsu_network_status.get_topic(f"{topic_syntax}n")
                dm_information[dmn_topic] = (
                    dm_protocol,
                    obu_rsu_network_status.position_netstat_dict)

    # ROSBAG topics
    ros_dict_information = topics.collect_topics_from_bag_file(
        Conf.ros2_files_path,
        Conf.ros2_topics,
        Conf.equivalent_topics)

    # Create output ROSBAG file
    create_bag_file(
        Conf.rosbag_output_directory_address,
        Conf.ros2_output_file_name,
        dm_information,
        ros_dict_information)

    # Create graphs of RSU
    # if Conf.rsu_only_graphs:
    #     for rsu in nodes_manager.get_rsu_nodes():
    #         rsu.graphs()

    # #
    # # creating csv and txt files from output rosbag file
    # if Conf.creating_txtFile_from_outputRos_file:
    #     txt_creator_from_rosbag(Conf.rosbag_output_directory_address + "/rosbag2_" + Conf.ros2_output_file_name)
    #
    # if Conf.csv_files:
    #     csv_creator_from_rosbag(Conf.report_output_directory_address, dm_information, ros_dict_information)
