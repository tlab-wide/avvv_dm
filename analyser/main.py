from ros2_interface.tools import register_new_types
from general_tools import creating_directory
from config_AVNV import Conf
import os


def main():
    #
    #
    # reading conf.ini file
    Conf()

    #
    #
    # creating output directories
    creating_directory(Conf.report_output_directory_address)
    creating_directory(Conf.rosbag_output_directory_address)

    #
    #
    # creating output dm topics rosbag2 file (and reports of that) from input csv and rosbag(2) files
    dm_merger()


if __name__ == "__main__":

    register_new_types(os.path.dirname(__file__) + "/dm_object_info_msgs/msg")  # registering new rosbag2 types
    register_new_types(os.path.dirname(__file__) + "/dm_freespace_info_msgs/msg")
    register_new_types(os.path.dirname(__file__) + "/dm_signal_info_msgs/msg")
    register_new_types(os.path.dirname(__file__) + "/dm_ros_msgs/msg")

    from ros2_pcap_merger import dm_merger

    main()
