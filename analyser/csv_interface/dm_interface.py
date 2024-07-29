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
    return int(dataframe_row[0])


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


def get_send_epochtime(dataframe_row) -> int:
    """
    This function gets each dataframe row of RSU or
    OBU and returns time of send or receive of that
    """
    return int(dataframe_row[-2])


def get_receive_epochtime(dataframe_row) -> int:
    """
    This function gets each dataframe row OBU and
    return time of receive of that
    """
    return int(dataframe_row[-1])


def get_rsu_id(dataframe_row) -> int:
    """this function gets the CSV row of obu and returns id of rsu that sent this DM message"""
    # TODO Correct this later
    # return int(dataframe_row.split(",")[0])
    return int(dataframe_row[0])


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
    send_epoch = get_send_epochtime(dataframe_row)
    id = ",".join([str(object_id), str(timestamp_its), str(send_epoch)])
    return id


def get_freespace_packet_id(dataframe_row) -> str:
    """
    Returns a unique ID for a given packet represented by the dataframe row
    """
    object_id = get_object_id(dataframe_row)
    timestamp_its = get_object_timestamp_its(dataframe_row)
    send_epoch = get_send_epochtime(dataframe_row)
    id = ",".join([str(object_id), str(timestamp_its), str(send_epoch)])
    return id


def get_signal_packet_id(dataframe_row) -> str:
    """
    Returns a unique ID for a given packet represented by the dataframe row
    """
    crp_id = get_signal_crp_id(dataframe_row)
    beacon_id = get_signal_beacon_id(dataframe_row)
    send_epoch = get_send_epochtime(dataframe_row)
    id = ",".join([str(crp_id), str(beacon_id), str(send_epoch)])
    return id
