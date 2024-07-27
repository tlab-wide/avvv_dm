"""
this module creates ros2 files
there is some function with different inputs
"""
from rosbags.serde import serialize_cdr
from rosbags.rosbag2 import Writer


def create_rosbag2_file_from_dmAndRos2_files(output_directory_address: str, ros_file_name: str, csv_dict_input: dict,
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
        for topic in csv_dict_input.keys():
            try:
                # Get message type from first message
                msgtype = csv_dict_input[topic][0][1].__msgtype__
                connection = writer.add_connection(topic, msgtype)
                # Add messages from this topic of CSV file dictionary to output rosbag2 file
                for msg_info in csv_dict_input[topic]:
                    try:
                        timestamp = msg_info[0]
                        msg = msg_info[1]

                        raw_data = serialize_cdr(msg, msg.__msgtype__)
                        writer.write(connection, timestamp, raw_data)
                    except Exception as message_error:
                        print(f'Error in writing our custom rosbag2 message. message : {msg_info[1]} and error : {message_error}')
            except Exception as topic_error:
                print(f'Error in custom rosbag2 topic. topic : {topic} and error : {topic_error}')

        # Write ROSBAG files information to output ROSBAG file
        for topic in ros_dict_input.keys():

            msgtype = ros_dict_input[topic][0][0].msgtype
            connection = writer.add_connection(topic, msgtype)

            for msg_info in ros_dict_input[topic]:
                # Get information from this message with above topic
                # connection = msg_info[0]
                timestamp = msg_info[1]
                raw_data = msg_info[2]
                # Write to output file
                writer.write(connection, timestamp, raw_data)
