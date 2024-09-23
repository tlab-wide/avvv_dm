"""
This module creates ROS2 messages
"""
from rosbags.typesys.types import builtin_interfaces__msg__Time as builtin_time
# dm_object_info_msgs
from rosbags.typesys.types import dm_object_info_msgs__msg__DistanceValue as DistanceValue
from rosbags.typesys.types import dm_object_info_msgs__msg__ControlSystemStates as ControlSystemStates
from rosbags.typesys.types import dm_object_info_msgs__msg__LaneLateralPosition as LaneLateralPosition
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectDimensionValue as ObjectDimensionValue
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectInfoArray as ObjectInfoArray
from rosbags.typesys.types import dm_object_info_msgs__msg__YawRateValue as YawRateValue
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectSize as ObjectSize
from rosbags.typesys.types import dm_object_info_msgs__msg__Location as Location
from rosbags.typesys.types import dm_object_info_msgs__msg__ExteriorLights as ExteriorLights
from rosbags.typesys.types import dm_object_info_msgs__msg__BrakeState as BrakeState
from rosbags.typesys.types import dm_object_info_msgs__msg__AltitudeAccuracy as AltitudeAccuracy
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectColor as ObjectColor
from rosbags.typesys.types import dm_object_info_msgs__msg__VehicleExtendedInformation as VehicleExtendedInformation
from rosbags.typesys.types import dm_object_info_msgs__msg__WGS84AngleAccuracy as WGS84AngleAccuracy
from rosbags.typesys.types import dm_object_info_msgs__msg__WGS84AngleValue as WGS84AngleValue
from rosbags.typesys.types import dm_object_info_msgs__msg__SemiAxisLength as SemiAxisLength
from rosbags.typesys.types import dm_object_info_msgs__msg__Altitude as Altitude
from rosbags.typesys.types import dm_object_info_msgs__msg__SpeedValue as SpeedValue
from rosbags.typesys.types import dm_object_info_msgs__msg__CrpId as CrpId
from rosbags.typesys.types import dm_object_info_msgs__msg__LanePosition as LanePosition
from rosbags.typesys.types import dm_object_info_msgs__msg__WGS84Angle as WGS84Angle
from rosbags.typesys.types import dm_object_info_msgs__msg__Speed as Speed
from rosbags.typesys.types import dm_object_info_msgs__msg__SubclassType as SubclassType
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectDimensionAccuracy as ObjectDimensionAccuracy
from rosbags.typesys.types import dm_object_info_msgs__msg__ClassId as ClassId
from rosbags.typesys.types import dm_object_info_msgs__msg__AccelerationAccuracy as AccelerationAccuracy
from rosbags.typesys.types import dm_object_info_msgs__msg__YawRateAccuracy as YawRateAccuracy
from rosbags.typesys.types import dm_object_info_msgs__msg__ReferencePoint as ReferencePoint
from rosbags.typesys.types import dm_object_info_msgs__msg__LaneCount as LaneCount
from rosbags.typesys.types import dm_object_info_msgs__msg__VehicleRole as VehicleRole
from rosbags.typesys.types import dm_object_info_msgs__msg__ShiftPosition as ShiftPosition
from rosbags.typesys.types import dm_object_info_msgs__msg__DistanceRatio as DistanceRatio
from rosbags.typesys.types import dm_object_info_msgs__msg__ExistenceConfidence as ExistenceConfidence
from rosbags.typesys.types import dm_object_info_msgs__msg__LaneId as LaneId
from rosbags.typesys.types import dm_object_info_msgs__msg__Longitude as Longitude
from rosbags.typesys.types import dm_object_info_msgs__msg__TimestampIts as TimestampIts
from rosbags.typesys.types import dm_object_info_msgs__msg__ThrottlePosition as ThrottlePosition
from rosbags.typesys.types import dm_object_info_msgs__msg__YawRate as YawRate
from rosbags.typesys.types import dm_object_info_msgs__msg__AccelerationValue as AccelerationValue
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectInfo as ObjectInfoMsg
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectDimension as ObjectDimension
from rosbags.typesys.types import dm_object_info_msgs__msg__Latitude as Latitude
from rosbags.typesys.types import dm_object_info_msgs__msg__AuxiliaryBrakeState as AuxiliaryBrakeState
from rosbags.typesys.types import dm_object_info_msgs__msg__ClassConfidence as ClassConfidence
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectClass as ObjectClass
from rosbags.typesys.types import dm_object_info_msgs__msg__SteeringAngle as SteeringAngle
from rosbags.typesys.types import dm_object_info_msgs__msg__SpeedAccuracy as SpeedAccuracy
from rosbags.typesys.types import dm_object_info_msgs__msg__OtherSubclassType as OtherSubclassType
from rosbags.typesys.types import dm_object_info_msgs__msg__GeodeticSystem as GeodeticSystem
from rosbags.typesys.types import dm_object_info_msgs__msg__ControlSystemState as ControlSystemState
from rosbags.typesys.types import dm_object_info_msgs__msg__Acceleration as Acceleration
from rosbags.typesys.types import dm_object_info_msgs__msg__ObjectId as ObjectId
# dm_signal_info_msgs
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalState as SignalState
from rosbags.typesys.types import dm_signal_info_msgs__msg__EventCount as EventCount
from rosbags.typesys.types import dm_signal_info_msgs__msg__SpecificControlFlags as SpecificControlFlags
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalLightInfo as SignalLightInfo
from rosbags.typesys.types import dm_signal_info_msgs__msg__ArrowLightIndication as ArrowLightIndication
from rosbags.typesys.types import dm_signal_info_msgs__msg__MaxTimeToChange as MaxTimeToChange
from rosbags.typesys.types import dm_signal_info_msgs__msg__MainLightIndication as MainLightIndication
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalId as SignalId
from rosbags.typesys.types import dm_signal_info_msgs__msg__MinTimeToChange as MinTimeToChange
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalInfoArray as SignalInfoArray
from rosbags.typesys.types import dm_signal_info_msgs__msg__CountDownStopFlag as CountDownStopFlag
from rosbags.typesys.types import dm_signal_info_msgs__msg__SignalInfo as SignalInfoMsg
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
        
        self.ros_time_stamp = builtin_time(
            sec=self.seconds,
            nanosec=self.nanoseconds)


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

        self.msg = NetworkStatusMsg(
            delay=delay,
            jitter=jitter,
            rssi=rssi,
            packet_loss=packet_loss,
            packet_count=packet_count,
            medium=medium,
            stamp=TimeStamp(time_stamp).ros_time_stamp)


class ObjectInfo:
    def __init__(self, dataframe_row):
        self.dataframe_row = dataframe_row
        self.__build_msg()
    
    def __build_msg(self):
        object_size = dm_interface.get_object_size(self.dataframe_row)
        self.msg = ObjectInfoMsg(
            id=ObjectId(
                value=dm_interface.get_object_id(self.dataframe_row)),
            time=TimestampIts(
                value=dm_interface.get_object_timestamp_its(self.dataframe_row)),
            object_class=[
                ObjectClass(
                    id=ClassId(
                        value=dm_interface.get_object_class_id(self.dataframe_row)),
                    confidence=ClassConfidence(
                        value=dm_interface.get_object_class_confidence(self.dataframe_row)),
                    subclass_type=None,
                    subclass_confidence=None)],
            existency=ExistenceConfidence(
                value=dm_interface.get_existence_confidence(self.dataframe_row)),
            object_location=Location(
                geodetic_system=GeodeticSystem(value=dm_interface.get_object_geodetic_system(self.dataframe_row)),
                latitude=Latitude(value=dm_interface.get_object_latitude(self.dataframe_row)),
                longitude=Longitude(value=dm_interface.get_object_longitude(self.dataframe_row)),
                altitude=Altitude(value=dm_interface.get_object_altitude(self.dataframe_row)),
                crp_id=None,
                dx_crp=None,
                dy_crp=None,
                dh_crp=None,
                lane_count=None,
                lane_position=None,
                lane_lateral_position=None,
                crp_id_begin=None,
                crp_id_end=None,
                lane_vertical_position=None,
                lane_id=None,
                dx_lane=None,
                dy_lane=None,
                dh_lane=None,
                semi_axis_length_major=None,
                semi_axis_length_minor=None,
                orientation=None,
                altitude_accuracy=None),
            ref_point=None,
            direction=None,
            speed=None,
            yaw_rate=None,
            acceleration=None,
            orientation=WGS84Angle(
                value=WGS84AngleValue(
                    value=dm_interface.get_object_orientation(self.dataframe_row)),
                accuracy=None),
            size=ObjectSize(
                length=ObjectDimension(
                    value=ObjectDimensionValue(value=object_size[0]),
                    accuracy=None),
                width=ObjectDimension(
                    value=ObjectDimensionValue(value=object_size[1]),
                    accuracy=None),
                height=ObjectDimension(
                    value=ObjectDimensionValue(value=object_size[2]),
                    accuracy=None)),
            color=None,
            shift_position=None,
            steering_angle_front=None,
            steering_angle_rear=None,
            brake_state=None,
            auxiliary_brake_state=None,
            throttle_position=None,
            exterior_lights=None,
            control_system_states=None,
            vehicle_role=None,
            vehicle_extended_info=None,
            towing_vehicle=None,
            information_source_list=None
        )

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
        signal_light_info = dm_interface.get_signal_light_info(self.dataframe_row)
        signal_light_info_msg = SignalLightInfo(
            main_light=MainLightIndication(value=signal_light_info[0]),
            arrow_light=ArrowLightIndication(value=signal_light_info[1]),
            min_time_to_change=MinTimeToChange(value=signal_light_info[2]),
            max_time_to_change=MaxTimeToChange(value=signal_light_info[3])
        )
        self.msg = SignalInfoMsg(
            crp_id=CrpId(value=dm_interface.get_signal_crp_id(self.dataframe_row)),
            signal_id_list=[SignalId(value=beacon_id) for beacon_id in dm_interface.get_signal_beacon_id(self.dataframe_row)],
            time=TimestampIts(value=dm_interface.get_signal_timestamp_its(self.dataframe_row)),
            state=None,
            specific_control_flags=None,
            event_count=None,
            count_down_stop_flag=None,
            signal_light_info_list=[signal_light_info_msg]
        )

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
        position_begin = dm_interface.get_freespace_position_begin(self.dataframe_row)
        position_end = dm_interface.get_freespace_position_end(self.dataframe_row)
        self.msg = FreespaceInfoMsg(
            id=ObjectId(
                value=dm_interface.get_object_id(self.dataframe_row)),
            time=TimestampIts(
                value=dm_interface.get_object_timestamp_its(self.dataframe_row)),
            existency=None,
            minimal_detectable_size=None,
            position_begin=Location(
                geodetic_system=GeodeticSystem(value=position_begin[0]),
                latitude=Latitude(value=position_begin[1]),
                longitude=Longitude(value=position_begin[2]),
                altitude=Altitude(value=position_begin[3]),
                crp_id=None,
                dx_crp=None,
                dy_crp=None,
                dh_crp=None,
                lane_count=None,
                lane_position=None,
                lane_lateral_position=None,
                crp_id_begin=None,
                crp_id_end=None,
                lane_vertical_position=None,
                lane_id=None,
                dx_lane=None,
                dy_lane=None,
                dh_lane=None,
                semi_axis_length_major=None,
                semi_axis_length_minor=None,
                orientation=None,
                altitude_accuracy=None),
            position_end=Location(
                geodetic_system=GeodeticSystem(value=position_end[0]),
                latitude=Latitude(value=position_end[1]),
                longitude=Longitude(value=position_end[2]),
                altitude=Altitude(value=position_end[3]),
                crp_id=None,
                dx_crp=None,
                dy_crp=None,
                dh_crp=None,
                lane_count=None,
                lane_position=None,
                lane_lateral_position=None,
                crp_id_begin=None,
                crp_id_end=None,
                lane_vertical_position=None,
                lane_id=None,
                dx_lane=None,
                dy_lane=None,
                dh_lane=None,
                semi_axis_length_major=None,
                semi_axis_length_minor=None,
                orientation=None,
                altitude_accuracy=None),
            length=None,
            id_begin=None,
            id_end=None,
            information_source_list=None
        )

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
