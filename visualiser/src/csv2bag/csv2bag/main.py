import os
import csv
import yaml

import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler
from rclpy.serialization import serialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from dm_freespace_info_msgs.msg import FreespaceInfoArray, FreespaceInfo
from dm_signal_info_msgs.msg import SignalInfoArray, SignalInfo, SignalId, SignalLightInfo, MainLightIndication, ArrowLightIndication, MinTimeToChange, MaxTimeToChange
from dm_object_info_msgs.msg import ObjectInfoArray, ObjectInfo, ObjectClass, ClassId, ClassConfidence, ExistenceConfidence


def main(args=None):
    # Initialise ROS client library
    rclpy.init(args=args)
    
    #############################
    ## Withdraw configurations ##
    #############################
    with open(
        os.path.join(
            get_package_share_directory(
                "csv2bag"),
            "config",
            "config.yaml")) as config_file:
        try:
            config = yaml.safe_load(config_file)
        except yaml.YAMLError as exc:
            print(exc)
            exit()
        
    # Input and output file paths
    INPUT_OBU_TF_PATH = config["paths"]["input_obu_tf_path"]
    INPUT_OBJECT_INFO_PATH = config["paths"]["input_obu_object_info_path"]
    INPUT_SIGNAL_INFO_PATH = config["paths"]["input_obu_signal_info_path"]
    INPUT_FREESPACE_INFO_PATH = config["paths"]["input_obu_freespace_info_path"]
    OUTPUT_PATH = config["paths"]["output_folder_path"]

    # Message topics
    OBU_TF_TOPIC_NAME = config["topics"]["obu_tf_topic_name"]
    OBJECT_INFO_TOPIC_NAME = config["topics"]["object_info_topic_name"]
    SIGNAL_INFO_TOPIC_NAME = config["topics"]["signal_info_topic_name"]
    FREESPACE_INFO_TOPIC_NAME = config["topics"]["freespace_info_topic_name"]

    # Indices of rows in CSV files
    # OBU TF
    OBU_X_IDX = config["indices"]["obu_x_idx"]
    OBU_Y_IDX = config["indices"]["obu_y_idx"]
    OBU_Z_IDX = config["indices"]["obu_z_idx"]
    OBU_YAW_IDX = config["indices"]["obu_yaw_idx"]
    OBU_STAMP_IDX = config["indices"]["obu_stamp_idx"]

    # Freespace Info
    FREESPACE_TIME_IDX = config["indices"]["freespace_time_idx"]
    FREESPACE_BEGIN_GEO_IDX = config["indices"]["freespace_begin_geo_idx"]
    FREESPACE_BEGIN_LAT_IDX = config["indices"]["freespace_begin_lat_idx"]
    FREESPACE_BEGIN_LON_IDX = config["indices"]["freespace_begin_lon_idx"]
    FREESPACE_BEGIN_ALT_IDX = config["indices"]["freespace_begin_alt_idx"]
    FREESPACE_END_GEO_IDX = config["indices"]["freespace_end_geo_idx"]
    FREESPACE_END_LAT_IDX = config["indices"]["freespace_end_lat_idx"]
    FREESPACE_END_LON_IDX = config["indices"]["freespace_end_lon_idx"]
    FREESPACE_END_ALT_IDX = config["indices"]["freespace_end_alt_idx"]
    FREESPACE_STAMP_IDX = config["indices"]["freespace_stamp_idx"]

    # Signal Info
    SIGNAL_CRP_IDX = config["indices"]["signal_crp_idx"]
    SIGNAL_BEACON_IDX = config["indices"]["signal_beacon_idx"]
    SIGNAL_TIME_IDX = config["indices"]["signal_time_idx"]
    SIGNAL_MAIN_IDX = config["indices"]["signal_main_idx"]
    SIGNAL_ARROW_IDX = config["indices"]["signal_arrow_idx"]
    SIGNAL_MIN_TIME_IDX = config["indices"]["signal_min_time_idx"]
    SIGNAL_MAX_TIME_IDX = config["indices"]["signal_max_time_idx"]
    SIGNAL_STAMP_IDX = config["indices"]["signal_stamp_idx"]

    # Object Info
    OBJECT_TIME_IDX = config["indices"]["object_time_idx"]
    OBJECT_CLASS_ID_IDX = config["indices"]["object_class_id_idx"]
    OBJECT_CLASS_CONF_IDX = config["indices"]["object_class_conf_idx"]
    OBJECT_EXIST_IDX = config["indices"]["object_exist_idx"]
    OBJECT_GEO_IDX = config["indices"]["object_geo_idx"]
    OBJECT_LAT_IDX = config["indices"]["object_lat_idx"]
    OBJECT_LON_IDX = config["indices"]["object_lon_idx"]
    OBJECT_ALT_IDX = config["indices"]["object_alt_idx"]
    OBJECT_ORIENT_IDX = config["indices"]["object_orient_idx"]
    OBJECT_LENGTH_IDX = config["indices"]["object_length_idx"]
    OBJECT_WIDTH_IDX = config["indices"]["object_width_idx"]
    OBJECT_HEIGHT_IDX = config["indices"]["object_height_idx"]
    OBJECT_STAMP_IDX = config["indices"]["object_stamp_idx"]

    # CSV timestamp scale (to ns)
    TIMESTAMP_SCALE = int(config["timestamp_scale"]) # ms to ns

    ###############################
    ## Create topics in BAG file ##
    ###############################
    tf_topic = rosbag2_py.TopicMetadata(
        name=OBU_TF_TOPIC_NAME,
        type="tf2_msgs/msg/TFMessage",
        serialization_format="cdr")

    freespace_topic = rosbag2_py.TopicMetadata(
        name=FREESPACE_INFO_TOPIC_NAME,
        type="dm_freespace_info_msgs/msg/FreespaceInfoArray",
        serialization_format="cdr")
    
    signal_topic = rosbag2_py.TopicMetadata(
        name=SIGNAL_INFO_TOPIC_NAME,
        type="dm_signal_info_msgs/msg/SignalInfoArray",
        serialization_format="cdr")
    
    object_topic = rosbag2_py.TopicMetadata(
        name=OBJECT_INFO_TOPIC_NAME,
        type="dm_object_info_msgs/msg/ObjectInfoArray",
        serialization_format="cdr")

    writer = rosbag2_py.SequentialWriter()

    writer.open(
        rosbag2_py.StorageOptions(uri=OUTPUT_PATH),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"))

    writer.create_topic(tf_topic)
    writer.create_topic(freespace_topic)
    writer.create_topic(signal_topic)
    writer.create_topic(object_topic)

    #################################
    ## Write data into ROSBAG file ##
    #################################
    print("Writing data to rosbag...")

    # Write tf data
    with open(INPUT_OBU_TF_PATH, "r") as csv_file:
        # Withdraw all data from the csv file
        csv_reader = csv.reader(csv_file)
        next(csv_reader, None) # Skip the header

        tf_message = TFMessage()
        tf_stamped = TransformStamped()
        tf_stamped.header.frame_id = "map"
        tf_stamped.child_frame_id = "base_link"
        
        for row in csv_reader:
            tf_message.transforms.clear()    
            tf_stamped.header.stamp.sec = int(int(row[OBU_STAMP_IDX]) / 1000)
            tf_stamped.header.stamp.nanosec = int(int(row[OBU_STAMP_IDX]) % 1000 * TIMESTAMP_SCALE)
            tf_stamped.transform.translation.x = float(row[OBU_X_IDX])
            tf_stamped.transform.translation.y = float(row[OBU_Y_IDX])
            tf_stamped.transform.translation.z = float(row[OBU_Z_IDX])
            q = quaternion_from_euler(0, 0, float(row[OBU_YAW_IDX]))
            tf_stamped.transform.rotation.x = q[0]
            tf_stamped.transform.rotation.y = q[1]
            tf_stamped.transform.rotation.z = q[2]
            tf_stamped.transform.rotation.w = q[3]
            tf_message.transforms.append(tf_stamped)
            writer.write(OBU_TF_TOPIC_NAME, serialize_message(tf_message), int(int(row[OBU_STAMP_IDX]) * TIMESTAMP_SCALE)) # Convert recorded time from ms to ns

    print("Finished writing tf data...")

    # Write freespace data
    with open(INPUT_FREESPACE_INFO_PATH, "r") as csv_file:
        # Withdraw all data from the csv file
        csv_reader = csv.reader(csv_file)
        next(csv_reader, None) # Skip the header

        freespace_array = FreespaceInfoArray()
        time = 0

        previous_received_time = 0

        for row in csv_reader:
            if int(row[FREESPACE_TIME_IDX]) != time and len(freespace_array.array):
                # Write the previous message into ROS bag
                writer.write(FREESPACE_INFO_TOPIC_NAME, serialize_message(freespace_array), int(previous_received_time * TIMESTAMP_SCALE)) # Convert recorded time from ms to ns
                time = int(row[FREESPACE_TIME_IDX])
                freespace_array.array.clear()

            freespace_info = FreespaceInfo()
            freespace_info.position_begin.geodetic_system.value = int(row[FREESPACE_BEGIN_GEO_IDX])
            freespace_info.position_begin.latitude.value = int(row[FREESPACE_BEGIN_LAT_IDX])
            freespace_info.position_begin.longitude.value = int(row[FREESPACE_BEGIN_LON_IDX])
            freespace_info.position_begin.altitude.value = int(row[FREESPACE_BEGIN_ALT_IDX])
            freespace_info.position_end.geodetic_system.value = int(row[FREESPACE_END_GEO_IDX])
            freespace_info.position_end.latitude.value = int(row[FREESPACE_END_LAT_IDX])
            freespace_info.position_end.longitude.value = int(row[FREESPACE_END_LON_IDX])
            freespace_info.position_end.altitude.value = int(row[FREESPACE_END_ALT_IDX])
            freespace_array.array.append(freespace_info)
            previous_received_time = int(row[FREESPACE_STAMP_IDX])

    print("Finished writing free space data...")

    # Write signal data
    with open(INPUT_SIGNAL_INFO_PATH, "r") as csv_file:
        # Withdraw all data from the csv file
        csv_reader = csv.reader(csv_file)
        next(csv_reader, None) # Skip the header

        signal_array = SignalInfoArray()
        time = 0

        previous_received_time = 0
        
        for row in csv_reader:
            if int(row[SIGNAL_TIME_IDX]) != time and len(signal_array.array):
                # Write the previous message into ROS bag
                writer.write(SIGNAL_INFO_TOPIC_NAME, serialize_message(signal_array), int(previous_received_time * TIMESTAMP_SCALE)) # Convert recorded time from ms to ns
                time = int(row[SIGNAL_TIME_IDX])
                signal_array.array.clear()

            signal_info = SignalInfo()
            signal_info.crp_id.value = int(row[SIGNAL_CRP_IDX])
            signal_info.signal_id_list = [SignalId(value=int(x)) for x in row[SIGNAL_BEACON_IDX].split(',')[:-1]]
            signal_info.signal_light_info_list = [
                SignalLightInfo(
                    main_light=MainLightIndication(value=int(float(row[SIGNAL_MAIN_IDX]))),
                    arrow_light=ArrowLightIndication(value=int(float(row[SIGNAL_ARROW_IDX]))),
                    min_time_to_change=MinTimeToChange(value=int(float(row[SIGNAL_MIN_TIME_IDX]))),
                    max_time_to_change=MaxTimeToChange(value=int(float(row[SIGNAL_MAX_TIME_IDX]))))
            ]
            signal_array.array.append(signal_info)
            previous_received_time = int(float(row[SIGNAL_STAMP_IDX]))

    print("Finished writing signal data...")

    # Write object data
    with open(INPUT_OBJECT_INFO_PATH, "r") as csv_file:
        # Withdraw all data from the csv file
        csv_reader = csv.reader(csv_file)
        next(csv_reader, None) # Skip the header

        object_array = ObjectInfoArray()
        time = 0

        previous_received_time = 0
        
        for row in csv_reader:
            if int(row[OBJECT_TIME_IDX]) != time and len(object_array.array):
                # Write the previous message into ROS bag
                writer.write(OBJECT_INFO_TOPIC_NAME, serialize_message(object_array), int(previous_received_time * TIMESTAMP_SCALE)) # Convert recorded time from ms to ns
                time = int(row[OBJECT_TIME_IDX])
                object_array.array.clear()

            object_info = ObjectInfo()
            object_info.object_class = [
                ObjectClass(
                    id=ClassId(value=int(row[OBJECT_CLASS_ID_IDX])),
                    confidence=ClassConfidence(value=int(row[OBJECT_CLASS_CONF_IDX])))
            ]

            object_info.existency = ExistenceConfidence(value=int(row[OBJECT_EXIST_IDX]))
            object_info.object_location.geodetic_system.value = int(row[OBJECT_GEO_IDX])
            object_info.object_location.latitude.value = int(row[OBJECT_LAT_IDX])
            object_info.object_location.longitude.value = int(row[OBJECT_LON_IDX])
            object_info.object_location.altitude.value = int(row[OBJECT_ALT_IDX])
            object_info.orientation.value.value = int(row[OBJECT_ORIENT_IDX])
            object_info.size.length.value.value = int(row[OBJECT_LENGTH_IDX])
            object_info.size.width.value.value = int(row[OBJECT_WIDTH_IDX])
            object_info.size.height.value.value = int(row[OBJECT_HEIGHT_IDX])
            object_array.array.append(object_info)
            previous_received_time = int(float(row[OBJECT_STAMP_IDX]))

    print("Finished writing object data...")

    del writer

    print("Operation successful!")

    rclpy.shutdown()


if __name__ == '__main__':
    main()