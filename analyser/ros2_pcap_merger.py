"""
this module is for merging pcap and rosbag2 files into one rosbag2 file and creating some reports for the topics, files, and network status
"""
import topics
from nodes import NodesManager
# from ros2_interface.ros2msg_gen import creating_small_cpm_ros2type_msg
# from ros2_interface.ros2file_read import csv_creator_from_rosbag, txt_creator_from_rosbag
from ros2_interface.ros2file_gen import create_rosbag2_file_from_pcapAndRos2_files
from config_AVNV import Conf
from network_status import NetworkStatus


# from packet_interface import cpm_interface


def dm_merger() -> None:
    """
    this is the primary function for merging csv files ( dm protocols) and ros2 files into one ros2 file with cpm-related topics
    this function also creates reports files for the files, topics, and network status
    :return:
    """

    csv_dict_information: dict = {}  # this dictionary saving information of pcap topics
    ros_dict_information: dict  # this dictionary saving information of rosbag2 topics

    dm_protocols = Conf.dm_protocols

    for dm_protocol in dm_protocols:

        rsu_files, obu_files = [], []

        try:
            if dm_protocol == "FreeSpaceInfo":
                rsu_files, obu_files = Conf.freeSpace_rsu_files, Conf.freeSpace_obu_files
            if dm_protocol == "ObjectInfo":
                rsu_files, obu_files = Conf.object_rsu_files, Conf.object_obu_files
            if dm_protocol == "SignalInfo":
                rsu_files, obu_files = Conf.signal_rsu_files, Conf.signal_obu_files

        except Exception as e:
            raise "can't read RSU or OBU files. error message : " + str(e)

        if len(rsu_files) == 0 or len(obu_files) == 0:
            raise "RSU or OBU files not found in config file!!!"

        #
        # reading csv files and creating Rsu and Obu objects
        nodes_manager = NodesManager(obu_files,
                                     rsu_files)  # todo : adding empty csv exception(when csv is empty we got error)

        # creating /rsu_#/CSV/SignalInfo, ObjectInfo and FreeSpaceInfo topics in final rosbag2
        for rsu in nodes_manager.get_rsu_nodes():
            rsu.set_dm_protocol_type(dm_protocol)

            topic, rsu_dm_msgs = rsu.get_csv_dm_info()
            csv_dict_information[topic] = rsu_dm_msgs

        #
        # creating /obu_#/rsu_#/network_status and /obu_#/rsu_#/cpmn topics in final rosbag2
        for obu in nodes_manager.get_obu_nodes():
            for rsu in nodes_manager.get_rsu_nodes():
                rsu_station_id = rsu.get_station_id()
                obu_id = obu.get_obu_id()
                sender_cap = rsu.get_csv_rows()
                receiver_cap = obu.get_obu_csv_rows_by_rsu_id(rsu_station_id)  # todo : this method not completed

                if len(sender_cap) == 0 or len(receiver_cap) == 0:
                    print(f"The number of rsu or obu packets specified in the protocol is zero.\tfiles name : {rsu.get_csv_file_name()}, {obu.get_csv_file_name()}")
                    continue

                obu_rsu_network_status_class = NetworkStatus(rsu_station_id, obu_id, sender_cap, receiver_cap, dm_protocol)

                # adding network status object to rsu object
                rsu.append_network_status(obu_rsu_network_status_class)

                # /obu_#/rsu_#/network_status topic generating
                netstat_topic = obu_rsu_network_status_class.get_topic(
                    dm_protocol + "/" + Conf.network_status_topic_name_syntax)
                ros2type_network_status_list = obu_rsu_network_status_class.get_ros2type_network_status_list()
                csv_dict_information[netstat_topic] = ros2type_network_status_list

                # /obu_#/rsu_#/cpmn topic generating
                cpmn_topic = obu_rsu_network_status_class.get_topic(dm_protocol + "/dmn")
                ros2type_cpmn_list = obu_rsu_network_status_class.get_ros2type_cpmn_list()
                csv_dict_information[cpmn_topic] = ros2type_cpmn_list

    #
    # rosbags topics
    ros_dict_information = topics.collect_topics_from_rosbag2_file(Conf.ros2_files_path, Conf.ros2_topics,
                                                                   Conf.equivalent_topics)

    #
    # creating output rosbag2 file
    create_rosbag2_file_from_pcapAndRos2_files(Conf.rosbag_output_directory_address, Conf.ros2_output_file_name,
                                               csv_dict_information, ros_dict_information)

    #
    # creating graphs of Rsu
    # if Conf.rsu_only_graphs:
    #     for rsu in nodes_manager.get_rsu_nodes():
    #         rsu.graphs()

    # #
    # # creating csv and txt files from output rosbag file
    # if Conf.creating_txtFile_from_outputRos_file:
    #     txt_creator_from_rosbag(Conf.rosbag_output_directory_address + "/rosbag2_" + Conf.ros2_output_file_name)
    #
    # if Conf.csv_files:
    #     csv_creator_from_rosbag(Conf.report_output_directory_address, pcap_dict_information, ros_dict_information)
