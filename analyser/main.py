from ros2_interface.tools import register_new_types
from general_tools import creating_directory
from config_avvv import Conf
import os


def main():
    # Read the conf.ini file
    Conf()

    # Create output directories
    creating_directory(Conf.report_output_directory_address)
    creating_directory(Conf.rosbag_output_directory_address)

    # Create output DM topics ROSBAG file and reports from input CSV and ROSBAG
    dm_merger()


if __name__ == "__main__":

    # List the paths to the DM message folders
    dm_base_path = os.path.join(
            os.environ.get("AVVV_DM_HOME"),
            "visualiser",
            "src",
            "dm_msgs")
    dm_msg_paths = [
        os.path.join(dm_base_path, "dm_object_info_msgs", "msg"),
        os.path.join(dm_base_path, "dm_freespace_info_msgs", "msg"),
        os.path.join(dm_base_path, "dm_signal_info_msgs", "msg"),
        os.path.join(dm_base_path, "dm_network_info_msgs", "msg")
    ]

    # Register the messages
    for path in dm_msg_paths:
        register_new_types(path)

    from ros2_dm_merger import dm_merger

    main()
