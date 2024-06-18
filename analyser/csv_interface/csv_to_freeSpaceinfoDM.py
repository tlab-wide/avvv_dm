"""this module is for creating ros2 freeSpaceInfo ros2 messages from freeSpaceInfo csv"""
from pathlib import Path
from typing import List

from .csv_general_tools import read_csv_file
from ros2_interface.ros2msg_gen import DMFreeSpaceInfo


def reading_free_space_info_csv(csv_file_address: Path) -> List[DMFreeSpaceInfo]:
    """this function getting csv file address and return a dictionary with send/receive time as key's and DM FreeSpaceInfoArray message as value"""
    csv_rows = read_csv_file(csv_file_address)

    dm_freespace_info_msgs = []
    for row in csv_rows:
        dm_freespace_info_msgs.append(DMFreeSpaceInfo(row))

    return dm_freespace_info_msgs

