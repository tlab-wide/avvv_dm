from typing import Tuple


def get_object_id(dataframe_row) -> int:
    """
    This function gets each (object_info or freespace_info)
    dataframe row of RSU or OBU and returns object ID in it
    """
    return int(dataframe_row[0], 16)


def get_object_timestamp_its(dataframe_row) -> int:
    """
    This function gets each (object_info or freespace_info)
    dataframe row of RSU or OBU and returns the ITS timestamp in it
    """
    return int(dataframe_row[1])


def get_object_class_id(dataframe_row) -> int:
    return int(dataframe_row[2])


def get_object_class_confidence(dataframe_row) -> int:
    return int(dataframe_row[3])


def get_existence_confidence(dataframe_row) -> int:
    return int(dataframe_row[18])


def get_object_geodetic_system(dataframe_row) -> int:
    return int(dataframe_row[19])


def get_object_latitude(dataframe_row) -> int:
    return int(dataframe_row[20])


def get_object_longitude(dataframe_row) -> int:
    return int(dataframe_row[21])


def get_object_altitude(dataframe_row) -> int:
    return int(dataframe_row[22])


def get_object_orientation(dataframe_row) -> int:
    return int(dataframe_row[50])


def get_object_size(dataframe_row) -> tuple[int]:
    """
    Return the object size as a tuple (length, width, height)
    """
    return int(dataframe_row[52]), int(dataframe_row[54]), int(dataframe_row[56])


def get_signal_crp_id_column() -> int:
    """
    Returns the column ID for CRP ID of the signal messages
    """
    return 0


def get_signal_crp_id(dataframe_row) -> int:
    """
    This function gets each (signal_info) dataframe row of
    RSU or OBU and returns object ID in it
    """
    return dataframe_row[0]


def get_signal_beacon_id(dataframe_row) -> list[int]:
    """
    This function gets each (signal_info) dataframe row of
    RSU or OBU and returns object ID in it
    """
    beacon_id_list = dataframe_row[1].split(",")
    beacon_id_list.pop(-1) # Remove the last one that always occurs empty and redundant
    return list(map(int, beacon_id_list))


def get_signal_timestamp_its(dataframe_row) -> int:
    """
    This function gets each (signal_info) dataframe row of RSU or
    OBU and returns the ITS timestamp in it
    """
    return int(dataframe_row[2])


def get_signal_light_info(dataframe_row) -> tuple[int]:
    """
    Return the first light information as tuple
    (main_light, arrow_light, min_time_to_change, max_time_to_change)
    """
    return dataframe_row[7], dataframe_row[8], dataframe_row[9], dataframe_row[10]

def get_generation_time(dataframe_row, row_length) -> int:
    """
    This function gets each dataframe row of RSU or
    OBU and returns generation time of that
    """
    return int(dataframe_row[row_length - 2])


def get_epochtime(dataframe_row, row_length) -> int:
    """
    This function gets each dataframe row and
    returns time of send or receive of that
    """
    return int(dataframe_row[row_length - 1])


def get_object_information_source_list_column() -> int:
    """
    Return the column ID for object information source list
    """
    return 77


def get_object_information_source_list(dataframe_row) -> list:
    """
    This function gets the CSV row of OBU and returns the information source
    list of the sensor that sent this DM message
    """
    return dataframe_row[77]


def get_freespace_position_begin(dataframe_row) -> tuple[int]:
    return dataframe_row[4], dataframe_row[5], dataframe_row[6], dataframe_row[7]


def get_freespace_position_end(dataframe_row) -> tuple[int]:
    return dataframe_row[26], dataframe_row[27], dataframe_row[28], dataframe_row[29]


def get_freespace_information_source_list_column() -> int:
    """
    Return the column ID for freespace information source list
    """
    return 53


def get_freespace_information_source_list(dataframe_row) -> list:
    """
    This function gets the CSV row of OBU and returns the information source
    list of the sensor that sent this DM message
    """
    return dataframe_row[53]


def get_rsu_position_in_xy(dataframe_row: str) -> Tuple[int, int]:
    """this function return position of rsu"""
    # todo : complete this function later
    return 0, 0


def get_object_packet_id(dataframe_row) -> str:
    """
    Returns a unique ID for a given packet represented by the dataframe row
    """
    object_id = get_object_id(dataframe_row)
    timestamp_its = get_object_timestamp_its(dataframe_row)
    send_epoch = get_generation_time(dataframe_row)
    id = ",".join([str(object_id), str(timestamp_its), str(send_epoch)])
    return id


def get_freespace_packet_id(dataframe_row) -> str:
    """
    Returns a unique ID for a given packet represented by the dataframe row
    """
    object_id = get_object_id(dataframe_row)
    timestamp_its = get_object_timestamp_its(dataframe_row)
    send_epoch = get_generation_time(dataframe_row)
    id = ",".join([str(object_id), str(timestamp_its), str(send_epoch)])
    return id


def get_signal_packet_id(dataframe_row) -> str:
    """
    Returns a unique ID for a given packet represented by the dataframe row
    """
    crp_id = get_signal_crp_id(dataframe_row)
    beacon_id = get_signal_beacon_id(dataframe_row)
    send_epoch = get_generation_time(dataframe_row)
    id = ",".join([str(crp_id), str(beacon_id), str(send_epoch)])
    return id
