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


def get_signal_crp_id(dataframe_row) -> int:
    """
    This function gets each (signal_info) dataframe row of
    RSU or OBU and returns object ID in it
    """
    return dataframe_row[0]


def get_signal_beacon_id(dataframe_row) -> int:
    """
    This function gets each (signal_info) dataframe row of
    RSU or OBU and returns object ID in it
    """
    return dataframe_row[1]


def get_signal_timestamp_its(dataframe_row) -> int:
    """
    This function gets each (signal_info) dataframe row of RSU or
    OBU and returns the ITS timestamp in it
    """
    return int(dataframe_row[2])


def get_generation_time(dataframe_row) -> int:
    """
    This function gets each dataframe row of RSU or
    OBU and returns generation time of that
    """
    return int(dataframe_row[-2])


def get_epochtime(dataframe_row) -> int:
    """
    This function gets each dataframe row OBU and
    return time of send or receive of that
    """
    return int(dataframe_row[-1])


def get_object_information_source_list(dataframe_row) -> int:
    """
    This function gets the CSV row of OBU and returns the information source
    list of the sensor that sent this DM message
    """
    return dataframe_row[77]


def get_freespace_information_source_list(dataframe_row) -> int:
    """
    This function gets the CSV row of OBU and returns the information source
    list of the sensor that sent this DM message
    """
    # TODO: Find the correct index for this
    return dataframe_row[77]


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
