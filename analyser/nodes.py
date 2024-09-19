import os
from abc import ABC
from typing import List

import pandas

from general_tools import creating_directory, merge_dicts, extract_id_from_csv_file_name
from network_status import NetworkStatus
from config_avvv import Conf
from plotting import Plotter
from csv_interface import csv_general_tools, dm_interface
from ros2_interface.ros2msg_gen import ObjectInfo, SignalInfo, FreespaceInfo


class Node(ABC):
    """
    This class is parent class for the RSU and OBU class
    """

    def __init__(self, csv_file_name: str):
        self._csv_file_name: str = csv_file_name
        self._its_station_id: str = self.__get_its_station_id()
        self._csv_rows = self.__read_csv_file()
        self._dm_protocol_type = None

    def __read_csv_file(self) -> pandas.DataFrame:
        """
        This function reading csv file and returning list of csv rows
        """
        rows = csv_general_tools.read_csv_file(os.path.join(Conf.csv_files_directory, self._csv_file_name))
        return rows

    def __get_its_station_id(self):
        try:
            file_name = self._csv_file_name
            # Split the filename by underscore to get the parts
            parts = file_name.split('_')
            # Extract the number before '.csv' from the last part
            station_id = int(parts[-1].split('.')[0])
            return station_id
        except Exception as e:
            raise f"Can't get the station id of {self.get_csv_file_name()}. Error message: {e}"
        
    def get_csv_rows(self) -> pandas.DataFrame:
        return self._csv_rows

    def get_csv_file_name(self) -> str:
        return self._csv_file_name

    def set_dm_protocol_type(self, dm_protocol_type: str) -> None:
        """what is type of DM protocol for this RSU"""
        self._dm_protocol_type = dm_protocol_type

    def get_dm_protocol_type(self) -> str:
        """return type of dm protocol"""
        return self._dm_protocol_type


class RSU(Node):
    def __init__(self, csv_file_address: str):
        super().__init__(csv_file_address)
        self.__rsu_position_x, self.__rsu_position_y = self.__get_rsu_position_in_xy()
        self.__network_statuses: List[NetworkStatus] = []
        self.__plotter = Plotter(self.get_plots_directory_name())
        
    def __get_rsu_position_in_xy(self):
        return Conf.rsu_info[str(self._its_station_id)]["xy"]

    def get_position(self):
        """
        this method return position of rsu station in x and y
        :return:
        """
        return self.__rsu_position_x, self.__rsu_position_y

    def get_station_id(self):
        """
        this method return station id of Rsu
        :return:
        """
        return self._its_station_id

    def get_csv_dm_info(self):
        """

        :return:
        """
        topic = self.get_topic_name()
        
        match self._dm_protocol_type:
            case "ObjectInfo":
                dm_message = ObjectInfo
                create_dm_message_array = ObjectInfo.create_object_info_array
            case "SignalInfo":
                dm_message = SignalInfo
                create_dm_message_array = SignalInfo.create_signal_info_array
            case "FreespaceInfo":
                dm_message = FreespaceInfo
                create_dm_message_array = FreespaceInfo.create_freespace_info_array
            case _:
                raise Exception("Protocol not specified when getting rows")

        msg_infos = []
        row_length = self._csv_rows.shape[1]
        generation_time_column = row_length - 2
        packet_message_groups = self._csv_rows.groupby(by=generation_time_column).groups
        
        for _, indices in packet_message_groups.items():
            dm_csv_row_message_time_stamp = dm_interface.get_epochtime(
                self._csv_rows.loc[indices[0]])
        
            dm_message_array = create_dm_message_array([
                dm_message(self._csv_rows.loc[index])
                for index in indices])

            msg_info = [
                dm_csv_row_message_time_stamp,
                dm_message_array]
        
            msg_infos.append(msg_info)

        return topic, msg_infos

    def get_topic_name(self):
        """
        Returns topic name of DM messages for this RSU
        """
        topic = f"/RSU_{str(self._its_station_id)}/{self._dm_protocol_type}"
        return topic

    def get_plots_directory_name(self) -> str:
        """
        this method creates and return the name of the plots directory of this Rsu in the graphs directory
        :return:
        """
        return f"RSU_{str(self._its_station_id)}"

    # def __netstat_position_graphs(self) -> None:
    #     """
    #     this method creates network status position graphs for this Rsu
    #     :return:
    #     """
    #     # getting all NetworkStatusPosition's dictionaries of this Rus
    #     netstat_position_dicts: List[Dict[float, PositionNetworkStatus]] = self.get_netstat_position_dicts()
    #
    #     # merging all NetworkStatusPosition's dictionaries to one NetworkStatusPosition dictionary
    #     netstat_position_dicts_merged = merge_dicts(netstat_position_dicts, overwrite=False, important_keys=False)
    #
    #     #
    #     # creating graphs
    #
    #     position_packetLoss_graph(netstat_position_dicts_merged, self.__plotter, self.__rsu_position_x,
    #                               self.__rsu_position_y, self.get_plots_directory_name())
    #
    #     position_delay_graph(netstat_position_dicts_merged, self.__plotter, self.__rsu_position_x,
    #                          self.__rsu_position_y, self.get_plots_directory_name())

    # def graphs(self) -> None:
    #     """
    #     this method creates all graphs of this Rsu
    #     :return:
    #     """
    #     self.__netstat_position_graphs()

    def append_network_status(self, netstat_obj: NetworkStatus) -> None:
        """
        This method is for adding new network status object to the list of
        network statuses
        :param netstat_obj: NetworkStatus object
        :return:
        """
        self.__network_statuses.append(netstat_obj)

    # def get_netstat_position_dicts(self) -> List[Dict[float, PositionNetworkStatus]]:
    #     """
    #     this method return list of position network status dictionaries
    #     :return:
    #     """
    #     return [netstat_obj.position_netstat_dict for netstat_obj in self.__network_statuses]
    #
    # def get_network_status_objects(self) -> List[NetworkStatus]:
    #     """
    #     this method return list of NetworkStatus objects
    #     :return: NetworkStatus objects list
    #     """
    #     return self.__network_statuses


class OBU(Node):
    def __init__(self, csv_file_name: str):
        super().__init__(csv_file_name)

    def get_obu_id(self) -> str:
        """
        This method returns OBU ID (using file name OBU CSV file)
        :return:
        """
        return self._its_station_id

    def get_obu_csv_rows_by_rsu_id(self, rsu_id: int) -> pandas.DataFrame:
        """
        This method returns CSV rows of this OBU by RSU ID
        """
        
        match self._dm_protocol_type:
            case "ObjectInfo":
                rsu_indicator_column = dm_interface.get_object_information_source_list_column()
                rsu_indicator = Conf.rsu_info[str(rsu_id)]["source_id_list"]
            case "FreespaceInfo":
                rsu_indicator_column = dm_interface.get_freespace_information_source_list_column()
                rsu_indicator = Conf.rsu_info[str(rsu_id)]["source_id_list"]
            case "SignalInfo":
                rsu_indicator_column = dm_interface.get_signal_crp_id_column()
                rsu_indicator = Conf.rsu_info[str(rsu_id)]["crp_id"]
            case _:
                raise Exception("Protocol not specified when getting rows by RSU ID")
        
        packet_message_groups = self._csv_rows.groupby(by=rsu_indicator_column).groups
        
        return self._csv_rows.loc[packet_message_groups[rsu_indicator]]
        

class NodesManager:
    """
    this class creates and manages all the Rsu and Obu classes
    """

    def __init__(self, obu_file_names: List[str], rsu_file_names: List[str]):
        self.__obu_file_names: List[str] = obu_file_names
        self.__rsu_file_names: List[str] = rsu_file_names
        self.__rsu_nodes: List[RSU] = self.__create_rsu_nodes()
        self.__obu_nodes: List[OBU] = self.__create_obu_nodes()
        # self.__nodes_reporter()

    def __create_rsu_nodes(self) -> List[RSU]:
        """
        this method creates rsu objects from rsu file names
        :return:
        """
        rsu_nodes_list: List[RSU] = []

        for rsu_csv_file_name in self.__rsu_file_names:
            rsu_nodes_list.append(RSU(rsu_csv_file_name))

        return rsu_nodes_list

    def __create_obu_nodes(self) -> List[OBU]:
        """
        this method creates obu objects from obu file names
        :return:
        """
        obu_nodes_list: List[OBU] = []

        for obu_csv_file_name in self.__obu_file_names:
            obu_nodes_list.append(OBU(obu_csv_file_name))

        return obu_nodes_list

    def __nodes_reporter(self) -> None:
        """
        This method creates a file report of nodes and saves it in the output directory
        :return:
        """
        # creating directory of Nodes_Reporters
        directory_address = Conf.report_output_directory_address + "/Nodes_Reports"
        creating_directory(directory_address)

        for rsu in self.__rsu_nodes:
            for obu in self.__obu_nodes:
                full_file_address = directory_address + "/" + rsu.get_csv_file_name().split(".")[0] + "-" + \
                                    obu.get_csv_file_name().split(".")[0] + ".txt"

                with open(full_file_address, 'w') as f:
                    obu_cap = obu.get_packets()
                    rsu_cap = rsu.get_packets()

                    f.write("Packets Report\n---------\n\n")
                    f.write("number of ITS packets in " + obu.get_csv_file_name() + " (obu) is: " + str(
                        len(list(obu_cap))) + "\n")
                    f.write("number of ITS packets in " + rsu.get_csv_file_name() + " (rsu) is: " + str(
                        len(list(rsu_cap))) + "\n\n")

                    f.write("---------\n")
                    f.write("ID = generationTime_timestamp(gnw)_rsuStationID\n\n")

                    f.write("number of different packet id in " + obu.get_csv_file_name() + " (obu) is: " + str(
                        len(obu.get_packet_id_dict())) + "\n")
                    f.write("number of different packet id in " + rsu.get_csv_file_name() + " (rsu) is: " + str(
                        len(rsu.get_packet_id_dict())) + "\n\n")

                    f.write("---------\n")

    def get_obu_nodes(self) -> List[OBU]:
        return self.__obu_nodes

    def get_rsu_nodes(self) -> List[RSU]:
        return self.__rsu_nodes

    def get_obu_file_names(self) -> List[str]:
        return self.__obu_file_names

    def get_rsu_file_names(self) -> List[str]:
        return self.__rsu_file_names
