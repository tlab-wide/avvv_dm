"""
This module categorizes raw packets and messages of ros2 files based on their topics
This category is in different forms, for example in dictionary(rsu cpm) and class(obu/rsu/network_status)
"""
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from rosbags.rosbag2 import Reader

if TYPE_CHECKING:
    from pathlib import Path


def get_standard_topic(src_file_name, topic, equivalent_topics: list) -> str:
    """
    this function creating standard topic ( standard topic in document) from source file name and
    message topic
    :param equivalent_topics:
    :param src_file_name: source file name
    :param topic: message topic in rosbag file
    :return:
    """
    for topics in equivalent_topics:
        if topic == topics[0]:
            topic = topics[1]
            break

    file_name = "/"+src_file_name.split("/")[-1]

    standard_topic = file_name + topic

    return standard_topic


def collect_topics_from_bag_file(src_list: list[Path], topics: list[str], equivalent_topics: list) -> dict:
    """collect topics from rosbag files

    Args:
        src_list: Source path list.
        topic: Name of topic .
        :param equivalent_topics: this is a list of topics that are equivalent to inputs ROSBAG files topics
        :param topics: topics in input ROSBAG files that must be merged with CSV files
        :param src_list: ROSBAG files
        :return dictionary like : { topic1:[connection, timestamp, rawdata], ...  }

    """
    ros2file_topic_information = {}  # this dictionary hold messages with input topics from input rosbag files
    for topic in topics:
        for src in src_list:
            with Reader(src) as reader:
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == topic:
                        standard_topic = get_standard_topic(src, topic, equivalent_topics)
                        if standard_topic in ros2file_topic_information.keys():
                            ros2file_topic_information[standard_topic].append([connection, timestamp, rawdata])
                        else:
                            ros2file_topic_information[standard_topic] = [[connection, timestamp, rawdata]]

    return ros2file_topic_information
