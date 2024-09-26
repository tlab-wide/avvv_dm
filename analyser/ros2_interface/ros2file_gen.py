"""
this module creates ros2 files
there is some function with different inputs
"""
from pandas import DataFrame
from rosbags.serde import serialize_cdr
from rosbags.rosbag2 import Writer

from config_avvv import Conf
from ros2_interface.ros2msg_gen import ObjectInfo, SignalInfo, FreespaceInfo, NetworkStatus
import csv_interface.dm_interface as dm_interface

def write_messages(
        writer: Writer,
        topic: str,
        msg_list: list) -> None:
    try:
        # Get message type from first message
        msgtype = msg_list[0][1].__msgtype__
        connection = writer.add_connection(topic, msgtype)
        # Add messages from this topic of CSV file dictionary to output rosbag2 file
        for msg_info in msg_list:
            try:
            
                timestamp = msg_info[0]
                msg = msg_info[1]
                raw_data = serialize_cdr(msg, msg.__msgtype__)                
                writer.write(connection, timestamp, raw_data)
            
            except Exception as message_error:
                print(f"""Error in writing our custom rosbag2 message.
                      message: {msg_info[1]} and error: {message_error}""")
    except Exception as topic_error:
        print(f"""Error in custom rosbag2 topic.
              topic : {topic} and error : {topic_error}""")
    
    print(f"Finished {topic}")
    print("########################################################################")
    print("########################################################################")
    print("########################################################################")
    print("########################################################################")


def write_netstat_messages(
        writer: Writer,
        topic: str,
        netstat_list: list[tuple]):
    
    netstat_messages = [
        (
            int(netstat[0] * 1e+9), # Timestamp to write in BAG file (convert to ns)
            NetworkStatus(*netstat[1]).msg
        )
        for netstat in netstat_list
    ]

    write_messages(writer, topic, netstat_messages)


def write_obu_messages(
        writer: Writer,
        topic: str,
        dm_protocol: str,
        position_netstat_dict: dict):
    
    match dm_protocol:
        case "ObjectInfo":
            DMMessage = ObjectInfo
            create_message_array = ObjectInfo.create_object_info_array
            create_message_array_n = ObjectInfo.create_object_info_array_n
        case "SignalInfo":
            DMMessage = SignalInfo
            create_message_array = SignalInfo.create_signal_info_array
            create_message_array_n = SignalInfo.create_signal_info_array_n
        case "FreespaceInfo":
            DMMessage = FreespaceInfo
            create_message_array = FreespaceInfo.create_freespace_info_array
            create_message_array_n = FreespaceInfo.create_freespace_info_array_n

    # Create DMN messages
    dmn_messages = [
        (
            int(dm_interface.get_epochtime(
                pn.rsu_packets.iloc[0]) * 1e+6), # Timestamp to write in BAG file (convert to ns from ms)
            create_message_array_n(
                create_message_array(
                    [
                        DMMessage(packet).msg
                        for packet in pn.rsu_packets.itertuples(index=False)
                    ]),
                NetworkStatus(
                    delay=pn.delay,
                    jitter=0, # TODO Not complete
                    rssi=255, # TODO not complete
                    packet_loss=pn.packet_loss,
                    packet_count=1,
                    time_stamp=timestamp,
                    medium=2)) # TODO Unknown medium type
        )
        for timestamp, pn in position_netstat_dict.items()
    ]

    write_messages(writer, topic, dmn_messages)


def write_rsu_messages(
        writer: Writer,
        topic: str,
        dm_protocol: str,
        all_csv_rows: DataFrame,
        csv_msg_groups: dict):
    
    rsu_messages = []

    match dm_protocol:
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
    for _, indices in csv_msg_groups.items():
        dm_csv_row_message_time_stamp = dm_interface.get_epochtime(
            all_csv_rows.loc[indices[0]])
    
        dm_message_array = create_dm_message_array([
            dm_message(all_csv_rows.loc[index]).msg
            for index in indices])

        msg_info = [
            int(dm_csv_row_message_time_stamp * 1e+6),  # Timestamp to write in BAG file (convert to ns from ms)
            dm_message_array]
    
        rsu_messages.append(msg_info)

    write_messages(writer, topic, rsu_messages)


def create_bag_file(
        output_directory_address: str,
        ros_file_name: str,
        csv_dict_input: dict,
        ros_dict_input: dict) -> None:
    """

    :param output_directory_address: address of outputs directory
    :param ros_file_name: name of output rosbag2 file

    :param csv_dict_input: { topic1:[[message1_timestamp,message1],...,[message(n)_timestamp,message(n)]] , ... ,
    topic(n):[[message1_timestamp,message1],...,[message(n)_timestamp,message(n)]] }

    :param ros_dict_input: dictionary that hold extracted messages with specific topics from rosbag files
    it is like : { topic1:[msg_info1 ,msg_info2, ... , msg_info(n) }
    and msg_info is list of three variable : [connection, timestamp, rawdata]

    :return:
    """
    # create writer instance and open for writing
    with Writer(output_directory_address + '/rosbag2_' + ros_file_name) as writer:

        # Write CSV files information to output rosbag2 file
        for topic in list(csv_dict_input.keys()):
            if "OBU" in topic:
                if Conf.network_status_topic_name_syntax in topic:
                    write_netstat_messages(
                        writer,
                        topic,
                        csv_dict_input[topic])
                else:
                    write_obu_messages(
                        writer,
                        topic,
                        csv_dict_input[topic][0],
                        csv_dict_input[topic][1])
            else:
                write_rsu_messages(
                    writer,
                    topic,
                    csv_dict_input[topic][0],
                    csv_dict_input[topic][1].get_csv_rows(),
                    csv_dict_input[topic][2])

            del csv_dict_input[topic]
            
        # Write ROSBAG files information to output ROSBAG file
        for topic in ros_dict_input.keys():

            msgtype = ros_dict_input[topic][0][0].msgtype
            connection = writer.add_connection(topic, msgtype)

            for msg_info in ros_dict_input[topic]:
                # Get information from this message with above topic
                timestamp = msg_info[1]
                raw_data = msg_info[2]
                # Write to output file
                writer.write(connection, timestamp, raw_data)
