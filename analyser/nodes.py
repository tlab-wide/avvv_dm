from general_tools import creating_directory, merge_dicts, extract_id_from_pcap_file_name
from network_status import NetworkStatus
from typing import List
from analyser.config_avvv import Conf
from abc import ABC
from plotting import Plotter
from csv_interface import csv_general_tools, dm_interface
from pathlib import Path
from ros2_interface.ros2msg_gen import DMCsvMessage


class Node(ABC):
    """
    this class is parent class for the rsu and abu class
    obu and rsu classes inherit from this class
    """

    def __init__(self, csv_file_name: str):
        self._csv_file_name: str = csv_file_name
        self._csv_rows: List = self.__reading_csv_file()
        self._dm_protocol_type = None

    def __reading_csv_file(self) -> List:
        """this function reading csv file and returning list of csv rows """
        rows = csv_general_tools.read_csv_file(Path(self._csv_file_name))
        return rows

    def get_csv_rows(self) -> List:
        return self._csv_rows

    def get_csv_file_name(self) -> str:
        return self._csv_file_name

    def set_dm_protocol_type(self, dm_protocol_type: str) -> None:
        """what is type of DM protocol for this RSU"""
        self._dm_protocol_type = dm_protocol_type

    def get_dm_protocol_type(self) -> str:
        """return type of dm protocol"""
        return self._dm_protocol_type


class Rsu(Node):
    def __init__(self, csv_file_address: str):
        super().__init__(csv_file_address)
        self.__rsu_position_x, self.__rsu_position_y = self.__get_rsu_position_in_xy()
        self.__rsu_station_id = self.__get_rsu_station_id()
        self.__network_statuses: List[NetworkStatus] = []
        self.__plotter = Plotter(self.get_plots_directory_name())

    def __get_rsu_station_id(self):
        try:
            file_name = self.get_csv_file_name()
            # Split the filename by underscore to get the parts
            parts = file_name.split('_')
            # Extract the number before '.csv' from the last part
            station_id = int(parts[-1].split('.')[0])
            return station_id
        except Exception as e:
            print(f'cant get the station id of {self.get_csv_file_name()}. error message : {e}')
            raise f'cant get the station id of {self.get_csv_file_name()}. error message : {e}'
    def __get_rsu_position_in_xy(self):
        return 0, 0  # todo

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
        return self.__rsu_station_id

    def get_csv_dm_info(self):
        """

        :return:
        """
        topic = self.get_topic_name()
        msg_infos = []
        for csv_row in self._csv_rows:
            dm_csv_row_message_time_stamp = dm_interface.get_epochtime(csv_row)
            msg_info = [dm_csv_row_message_time_stamp,
                        DMCsvMessage(csv_row, dm_csv_row_message_time_stamp).ros_csv_dm_message]
            msg_infos.append(msg_info)

        return topic, msg_infos

    def get_topic_name(self):
        """return topic name of dm messages for this rsu"""
        topic = "/rsu_" + str(self.__rsu_station_id) + "/CSV/" + self._dm_protocol_type
        return topic

    def get_plots_directory_name(self) -> str:
        """
        this method creates and return the name of the plots directory of this Rsu in the graphs directory
        :return:
        """
        return "RSU_" + str(self.__rsu_station_id)

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
        this method for adding new network status object to the list of network statuses
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


class Obu(Node):
    def __init__(self, pcap_file_name: str):
        super().__init__(pcap_file_name)

    def get_obu_id(self) -> str:
        """
        this method return id of obu ( with using file name obu pcap file )
        :return:
        """
        return extract_id_from_pcap_file_name(self.get_csv_file_name())

    def get_obu_csv_rows_by_rsu_id(self, rsu_id: int) -> List:
        """this method return csv rows of this obu by obu_id"""
        output_list = []
        for csv_row in self.get_csv_rows():
            # if dm_interface.get_rsu_id(csv_row) == rsu_id: # todo : adding this condition later
            output_list.append(csv_row)
        return output_list


class NodesManager:
    """
    this class creates and manages all the Rsu and Obu classes
    """

    def __init__(self, obu_file_names: List[str], rsu_file_names: List[str]):
        self.__obu_file_names: List[str] = obu_file_names
        self.__rsu_file_names: List[str] = rsu_file_names
        self.__rsu_nodes: List[Rsu] = self.__create_rsu_nodes()
        self.__obu_nodes: List[Obu] = self.__create_obu_nodes()
        # self.__nodes_reporter()

    def __create_rsu_nodes(self) -> List[Rsu]:
        """
        this method creates rsu objects from rsu file names
        :return:
        """
        rsu_nodes_list: List[Rsu] = []

        for rsu_csv_file_name in self.__rsu_file_names:
            rsu_nodes_list.append(Rsu(rsu_csv_file_name))

        return rsu_nodes_list

    def __create_obu_nodes(self) -> List[Obu]:
        """
        this method creates obu objects from obu file names
        :return:
        """
        obu_nodes_list: List[Obu] = []

        for obu_csv_file_name in self.__obu_file_names:
            obu_nodes_list.append(Obu(obu_csv_file_name))

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
                    f.write("number of ITS packets in " + obu.get_pcap_file_name() + " (obu) is: " + str(
                        len(list(obu_cap))) + "\n")
                    f.write("number of ITS packets in " + rsu.get_pcap_file_name() + " (rsu) is: " + str(
                        len(list(rsu_cap))) + "\n\n")

                    f.write("---------\n")
                    f.write("ID = generationTime_timestamp(gnw)_rsuStationID\n\n")

                    f.write("number of different packet id in " + obu.get_pcap_file_name() + " (obu) is: " + str(
                        len(obu.get_packet_id_dict())) + "\n")
                    f.write("number of different packet id in " + rsu.get_pcap_file_name() + " (rsu) is: " + str(
                        len(rsu.get_packet_id_dict())) + "\n\n")

                    f.write("---------\n")

    def get_obu_nodes(self) -> List[Obu]:
        return self.__obu_nodes

    def get_rsu_nodes(self) -> List[Rsu]:
        return self.__rsu_nodes

    def get_obu_file_names(self) -> List[str]:
        return self.__obu_file_names

    def get_rsu_file_names(self) -> List[str]:
        return self.__rsu_file_names
