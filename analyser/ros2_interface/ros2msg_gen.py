"""
This module creates ros2 messages
"""
from rosbags.typesys.types import builtin_interfaces__msg__Time as builtin_time
from rosbags.typesys.types import dm_network_info_msgs__msg__NetworkStatus as NetworkStatusMessage
from rosbags.typesys.types import dm_network_info_msgs__msg__ObjectInfoN as ObjectInfoN
from rosbags.typesys.types import dm_network_info_msgs__msg__FreespaceInfoN as FreespaceInfoN
from rosbags.typesys.types import dm_network_info_msgs__msg__SignalInfoN as SignalInfoN
from datetime import datetime
from csv_interface import dm_interface


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


class DM:
    def __init__(self, csv_row, time_stamp):
        self.csv_row = csv_row
        self.ros_time_stamp = TimeStamp(time_stamp).ros_time_stamp