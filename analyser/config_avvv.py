import os
from configparser import ConfigParser
import json

import pandas as pd

from csv_interface.dm_interface import get_signal_crp_id_column, get_object_information_source_list_column, get_freespace_information_source_list_column


def get_station_id_from_filename(filename: str) -> str:
    # Split the filename by underscore to get the parts
    parts = filename.split('_')
    # Extract the number before '.csv' from the last part
    station_id = parts[-1].split('.')[0]
    return station_id
    

class Conf:
    """
    this is config class
    """
    # ros2 section
    ros2_files_directory: str
    ros2_files_path: list = []  # this variable is not in config file
    ros2_topics: list

    # rsu positions
    rsu_xy_positions: dict

    # csv files section
    dm_protocols: list
    csv_files_directory: str
    signal_rsu_files: list = []
    signal_obu_files: list = []
    freespace_rsu_files: list = []
    freespace_obu_files: list = []
    object_rsu_files: list = []
    object_obu_files: list = []

    # output section
    report_output_directory_address: str
    rosbag_output_directory_address: str
    ros2_output_file_name: str
    cpm_topic_name_syntax: str  # not yet completed
    cpmn_topic_name_syntax: str
    network_status_topic_name_syntax: str
    equivalent_topics: list
    network_status_time: int

    # advance
    simulation: bool
    creating_txtFile_from_outputRos_file: bool
    graphs_style: str
    time_reporter: bool
    position_reporter: bool
    distance_reporters: bool
    showing_graphs: bool
    save_graphs_image: bool
    save_graphs_pdf: bool
    save_graphs_csv: bool
    save_graphs_pickle: bool
    position_reporter_grid_x_size: float
    position_reporter_grid_y_size: float
    position_reporter_grid_z_size: float
    csv_files: bool
    max_difference_time_for_equivalent_tf_message: float
    rsu_effective_distance: float
    online_graph_display_speed: float
    rsu_only_graphs: bool

    # position_reporter_packet_loss
    packet_loss_color: str
    packet_color: str
    just_show_packet_loss: bool
    packet_loss_point_size: float
    packet_point_size: float

    # position_delay reporter
    position_delay_packet_point_size: float

    # csv files
    csv_topics: list
    cpm_format_perceived_objects_number: int

    def __init__(self, config_file_name: str = "conf.ini"):
        """

        :param config_file_name:
        :return:
        """

        # reading information from config file
        configuration = ConfigParser()
        configuration.read(os.path.dirname(__file__) + "/" + config_file_name)

        # reading information about CSV files
        Conf.csv_files_directory = json.loads(configuration.get('csv', 'csv_files_directory'))
        for file in os.listdir(Conf.csv_files_directory):
            if "rsu" in file.lower():
                if "signal" in file.lower():
                    Conf.signal_rsu_files.append(file)
                elif "free" in file.lower():
                    Conf.freespace_rsu_files.append(file)
                elif "object" in file.lower():
                    Conf.object_rsu_files.append(file)
            elif "obu" in file.lower():
                if "signal" in file.lower():
                    Conf.signal_obu_files.append(file)
                elif "free" in file.lower():
                    Conf.freespace_obu_files.append(file)
                elif "object" in file.lower():
                    Conf.object_obu_files.append(file)

        Conf.dm_protocols = json.loads(configuration.get('csv', 'dm_protocols'))

        # reading information about ROSBAG files
        Conf.ros2_files_directory = json.loads(configuration.get('ros2', 'rosbag_files_directory'))
        Conf.ros2_topics = json.loads(configuration.get('ros2', 'topics'))

        # Read and automatically withdraw RSU information
        Conf.rsu_info = json.loads(configuration.get('rsus', 'rsu_info'))

        # Automatically withdraw RSU CRP_IDs for signals
        # and information_source_lists both for objects and freespaces
        for filename in Conf.object_rsu_files:
            station_id = get_station_id_from_filename(filename)
            df = pd.read_csv(os.path.join(Conf.csv_files_directory, filename), header=None)
            information_source_list = df.iloc[0][get_object_information_source_list_column()]
            # We assume information_source_list is the same for objects and freespaces
            Conf.rsu_info[station_id]["information_source_list"] = information_source_list
                
        for filename in Conf.signal_rsu_files:
            station_id = get_station_id_from_filename(filename)
            df = pd.read_csv(os.path.join(Conf.csv_files_directory, filename), header=None)
            crp_id = df.iloc[0][get_signal_crp_id_column()]
            Conf.rsu_info[station_id]["crp_id"] = crp_id

        # Read information about output file
        Conf.report_output_directory_address = json.loads(configuration.get('output', 'report_output_address'))
        Conf.rosbag_output_directory_address = json.loads(configuration.get('output', 'rosbag_output_address'))
        Conf.ros2_output_file_name = json.loads(configuration.get('output', 'file_name'))
        Conf.network_status_topic_name_syntax = json.loads(configuration.get('output', 'network_status_topic_name_syntax'))
        Conf.equivalent_topics = json.loads(configuration.get('output', 'topics_equivalent_to_ros2_topics'))
        Conf.network_status_time = json.loads(configuration.get('output', 'network_status_time'))

        # reading information of advance in config file
        Conf.simulation = Conf.boolean(json.loads(configuration.get("advance", "simulation")))
        Conf.graphs_style = json.loads(configuration.get("advance", "graphs_style"))
        Conf.creating_txtFile_from_outputRos_file = Conf.boolean(json.loads(
            configuration.get('advance', 'creating_txtFile_from_outputRos_file')))
        Conf.time_reporter = Conf.boolean(json.loads(configuration.get("advance", "time_reporter")))
        Conf.distance_reporters = Conf.boolean(json.loads(configuration.get("advance", "distance_reporters")))
        Conf.rsu_only_graphs = Conf.boolean(json.loads(configuration.get("advance", "rsu_only_graphs")))
        Conf.save_graphs_image = Conf.boolean(json.loads(configuration.get("advance", "save_graphs_image")))
        Conf.save_graphs_csv = Conf.boolean(json.loads(configuration.get("advance", "save_graphs_csv")))
        Conf.save_graphs_pdf = Conf.boolean(json.loads(configuration.get("advance", "save_graphs_pdf")))
        Conf.save_graphs_pickle = Conf.boolean(json.loads(configuration.get("advance", "save_graphs_pickle")))
        Conf.showing_graphs = Conf.boolean(json.loads(configuration.get("advance", "showing_graphs")))
        Conf.position_reporter_grid_x_size = json.loads(configuration.get("advance", "position_reporter_grid_x_size"))
        Conf.position_reporter_grid_y_size = json.loads(configuration.get("advance", "position_reporter_grid_y_size"))
        Conf.position_reporter_grid_z_size = json.loads(configuration.get("advance", "position_reporter_grid_z_size"))
        Conf.position_reporter = Conf.boolean(json.loads(configuration.get("advance", "position_reporter")))
        Conf.csv_files = Conf.boolean(json.loads(configuration.get("advance", "csv_files")))
        Conf.rsu_effective_distance = json.loads(configuration.get("advance", "rsu_effective_distance"))
        Conf.max_difference_time_for_equivalent_tf_message = json.loads(
            configuration.get("advance", "max_difference_time_for_equivalent_tf_message"))
        Conf.online_graph_display_speed = json.loads(configuration.get("advance", "online_graph_display_speed"))
        for path in os.listdir(Conf.ros2_files_directory):
            Conf.ros2_files_path.append(Conf.ros2_files_directory + "/" + path)

        # reading information of position_reporter_packet_loss section in config file
        Conf.packet_loss_color = json.loads(configuration.get("position_reporter_packet_loss", "packet_loss_color"))
        Conf.packet_color = json.loads(configuration.get("position_reporter_packet_loss", "packet_color"))
        Conf.just_show_packet_loss = Conf.boolean(
            json.loads(configuration.get("position_reporter_packet_loss", "just_show_packet_loss")))
        Conf.packet_loss_point_size = json.loads(
            configuration.get("position_reporter_packet_loss", "packet_loss_point_size"))
        Conf.packet_point_size = json.loads(configuration.get("position_reporter_packet_loss", "packet_point_size"))

        # reading information of position_reporter_delay section in config file
        Conf.position_delay_packet_point_size = json.loads(configuration.get("position_reporter_delay", "packet_size"))

        # csv section
        Conf.csv_topics = json.loads(configuration.get("csv_files", "csv_topics"))
        Conf.cpm_format_perceived_objects_number = json.loads(
            configuration.get("csv_files", "cpm_format_perceived_objects_number"))

    @staticmethod
    def boolean(value: str) -> bool:
        return value.lower() == "true"
