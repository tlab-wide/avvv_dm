"""
This module creates ROS2 messages
"""
from rosbags.typesys.types import builtin_interfaces__msg__Time as builtin_time

# dm_object_info_msgs
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectInfoArray as ObjectInfoArray
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectInfo as ObjectInfoMsg
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectClass as ObjectClass

# dm_signal_info_msgs
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalInfoArray as SignalInfoArray
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalInfo as SignalInfoMsg
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalId as SignalId
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalLightInfo as SignalLightInfo

# dm_freespace_info_msgs
from rosbags.typesys.types import dm_freespace_info_msgs__msg__FreespaceInfoArray as FreespaceInfoArray
from rosbags.typesys.types import dm_freespace_info_msgs__msg__FreespaceInfo as FreespaceInfoMsg

# dm_network_info_msgs
from rosbags.typesys.types import dm_network_info_msgs__msg__NetworkStatus as NetworkStatusMsg
from rosbags.typesys.types import dm_network_info_msgs__msg__ObjectInfoN as ObjectInfoNMsg
from rosbags.typesys.types import dm_network_info_msgs__msg__FreespaceInfoN as FreespaceInfoNMsg
from rosbags.typesys.types import dm_network_info_msgs__msg__SignalInfoN as SignalInfoNMsg
from csv_interface import dm_interface


class TimeStamp:
    def __init__(self, time_stamp: int):
        self.raw_time_stamp = time_stamp
        self.seconds = time_stamp // 1000
        self.nanoseconds = int((time_stamp % 1000) * 1e6)  # 1e6 nanoseconds in a millisecond
        self.ros_time_stamp = builtin_time(sec=self.seconds, nanosec=self.nanoseconds)


class NetworkStatus:
    def __init__(
            self,
            delay,
            jitter,
            rssi,
            packet_loss,
            packet_count,
            time_stamp,
            medium):
        self.msg = NetworkStatusMsg()
        self.msg.delay = delay
        self.msg.jitter = jitter
        self.msg.rssi = rssi
        self.msg.packet_loss = packet_loss
        self.msg.packet_count = packet_count
        self.msg.medium = medium 
        self.msg.stamp = TimeStamp(time_stamp).ros_time_stamp


class ObjectInfo:
    def __init__(self, dataframe_row):
        self.dataframe_row = dataframe_row
        self.__build_msg()
    
    def __build_msg(self):
        self.msg = ObjectInfoMsg()
        self.msg.id.value = dm_interface.get_object_id(self.dataframe_row)
        self.msg.time.value = dm_interface.get_object_timestamp_its(self.dataframe_row)
        object_class = ObjectClass()
        object_class.id.value = dm_interface.get_object_class_id(self.dataframe_row)
        object_class.confidence.value = dm_interface.get_object_class_confidence(self.dataframe_row)
        self.msg.object_class.append(object_class)
        self.msg.existency.value = dm_interface.get_class_existence_confidence(self.dataframe_row)
        self.msg.object_location.geodetic_system.value = dm_interface.get_object_geodetic_system(self.dataframe_row)
        self.msg.object_location.latitude.value = dm_interface.get_object_latitude(self.dataframe_row)
        self.msg.object_location.longitude.value = dm_interface.get_object_longitude(self.dataframe_row)
        self.msg.object_location.altitude.value = dm_interface.get_object_altitude(self.dataframe_row)
        self.msg.orientation.value = dm_interface.get_object_orientation(self.dataframe_row)
        object_size = dm_interface.get_object_size(self.dataframe_row)
        self.msg.size.length.value.value = object_size[0]
        self.msg.size.width.value.value = object_size[1]
        self.msg.size.height.value.value = object_size[2]

    @staticmethod
    def create_object_info_array(object_infos: list[ObjectInfoMsg]) -> ObjectInfoArray:
        object_info_array = ObjectInfoArray(array=object_infos)
        return object_info_array

    @staticmethod
    def create_object_info_array_n(
            object_info_array: ObjectInfoArray,
            network_status: NetworkStatus) -> ObjectInfoNMsg:
        object_info_n = ObjectInfoNMsg(
            object_info_array=object_info_array,
            network_status=network_status)
        return object_info_n


class SignalInfo:
    def __init__(self, dataframe_row):
        self.dataframe_row = dataframe_row
        self.__build_msg()
    
    def __build_msg(self):
        self.msg = SignalInfoMsg()
        self.msg.crp_id.value = dm_interface.get_signal_crp_id(self.dataframe_row)
        self.msg.signal_id_list.append(
            [SignalId(value=beacon_id) for beacon_id in dm_interface.get_signal_beacon_id(self.dataframe_row)])
        self.msg.time.value = dm_interface.get_signal_timestamp_its(self.dataframe_row)
        signal_light_info = dm_interface.get_signal_light_info(self.dataframe_row)
        signal_light_info_msg = SignalLightInfo()
        signal_light_info_msg.main_light.value = signal_light_info[0]
        signal_light_info_msg.arrow_light.value = signal_light_info[1]
        signal_light_info_msg.min_time_to_change.value = signal_light_info[2]
        signal_light_info_msg.max_time_to_change.value = signal_light_info[3]
        self.msg.signal_light_info_list.append(signal_light_info_msg)

    @staticmethod
    def create_signal_info_array(signal_infos: list[SignalInfoMsg]) -> SignalInfoArray:
        signal_info_array = SignalInfoArray(array=signal_infos)
        return signal_info_array
    
    @staticmethod
    def create_signal_info_array_n(
            signal_info_array: SignalInfoArray,
            network_status: NetworkStatus) -> SignalInfoNMsg:
        signal_info_n = SignalInfoNMsg(
            signal_info_array=signal_info_array,
            network_status=network_status)
        return signal_info_n


class FreespaceInfo:
    def __init__(self, dataframe_row):
        self.dataframe_row = dataframe_row
        self.__build_msg()
    
    def __build_msg(self):
        self.msg = FreespaceInfoMsg()
        self.msg.id.value = dm_interface.get_object_id(self.dataframe_row)
        self.msg.time.value = dm_interface.get_object_timestamp_its(self.dataframe_row)
        position_begin = dm_interface.get_freespace_position_begin(self.dataframe_row)
        position_end = dm_interface.get_freespace_position_end(self.dataframe_row)
        self.msg.position_begin.geodetic_system.value = position_begin[0]
        self.msg.position_begin.latitude.value = position_begin[1]
        self.msg.position_begin.longitude.value = position_begin[2]
        self.msg.position_begin.altitude.value = position_begin[3]
        self.msg.position_end.geodetic_system.value = position_end[0]
        self.msg.position_end.latitude.value = position_end[1]
        self.msg.position_end.longitude.value = position_end[2]
        self.msg.position_end.altitude.value = position_end[3]

    @staticmethod
    def create_freespace_info_array(freespace_infos: list[FreespaceInfoMsg]) -> FreespaceInfoArray:
        freespace_info_array = FreespaceInfoArray(array=freespace_infos)
        return freespace_info_array
    
    @staticmethod
    def create_freespace_info_array_n(
            freespace_info_array: FreespaceInfoArray,
            network_status: NetworkStatus) -> FreespaceInfoNMsg:
        freespace_info_n = FreespaceInfoNMsg(
            freespace_info_array=freespace_info_array,
            network_status=network_status)
        return freespace_info_n
