"""
This module creates ros2 messages
"""
from rosbags.typesys.types import builtin_interfaces__msg__Time as builtin_time
from rosbags.typesys.types import std_msgs__msg__String as RosString
from rosbags.typesys.types import dm_ros_msgs__msg__DMMessage as DMMessage
from rosbags.typesys.types import dm_ros_msgs__msg__DMN as DMNMessage
from rosbags.typesys.types import dm_ros_msgs__msg__NetworkStatus as NetworkStatusMessage
from datetime import datetime

from csv_interface import dm_interface


#
# from rosbags.typesys.types import dm_freespace_info_msgs__msg__CsvFreespaceInfo as CsvFreeSpaceInfo
#
#
# class DMObjectInfo:
#     """
#     DM_Object_Info class
#     """
#
#     def __init__(self, dm_object_info_csv_standard_row):
#         self.csv_dm_row = dm_object_info_csv_standard_row
#
#
# class DMFreeSpaceInfo:
#     def __init__(self, dm_free_space_info_csv_standard_row):
#         self.csv_dm_row = dm_free_space_info_csv_standard_row
#         self.csv_free_space_info_ros_msg = CsvFreeSpaceInfo(data=str(self.csv_dm_row), first_time=self.csv_dm_row[-1],
#                                                             second_time=self.csv_dm_row[-2])
#
#     def get_csv_free_space_info_ros_msg(self) -> CsvFreeSpaceInfo:
#         return self.csv_free_space_info_ros_msg


class TimeStamp:
    def __init__(self, time_stamp: int):
        self.raw_time_stamp = time_stamp
        self.seconds = time_stamp // 1000
        self.nanoseconds = int((time_stamp % 1000) * 1e6)  # 1e6 nanoseconds in a millisecond
        self.ros_time_stamp = builtin_time(sec=self.seconds, nanosec=self.nanoseconds)


class NetworkStatus:
    def __init__(self, delay, jitter, rssi, packet_loss, packet_count, time_stamp):
        self.delay = delay
        self.jitter = jitter
        self.rssi = rssi
        self.packet_loss = packet_loss
        self.packet_count = packet_count
        self.ros_time_stamp = TimeStamp(time_stamp).ros_time_stamp
        self.network_status = NetworkStatusMessage(delay=delay, jitter=jitter, rssi=rssi, packet_loss=packet_loss,
                                                   packet_count=packet_count, stamp=self.ros_time_stamp)


class DMCsvMessage:
    def __init__(self, csv_row, time_stamp):
        self.csv_row = RosString(data=csv_row)
        self.ros_time_stamp = TimeStamp(time_stamp).ros_time_stamp
        self.ros_csv_dm_message = DMMessage(csv_row=self.csv_row, stamp=self.ros_time_stamp)


class DMNCsvMessage:

    def __init__(self, dm: DMCsvMessage, network_status: NetworkStatusMessage, obu_tf=None):
        self.obu_tf = obu_tf
        self.dm = dm
        self.network_status = network_status
        self.ros_csv_dmn_message = DMNMessage(dm=self.dm, network_status=self.network_status)
