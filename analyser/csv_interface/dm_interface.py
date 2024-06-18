from typing import Tuple


def get_epochtime(csv_row: str) -> int:
    """this function getting each csv row of RSU or OBU and return time of send or receive of that """
    # Correct this later
    # return int(csv_row.split(",")[-1])
    return int(csv_row[-1])


def get_timestamp(csv_row: str, rsu: bool) -> int:
    """this function getting each csv row of RSU or OBU and return time of send or receive of that """
    # TODO Correct this later
    # if rsu:
    #     return int(csv_row.split(",")[-1])
    # return int(csv_row.split(",")[-2])
    return int(csv_row[-2])


def get_rsu_id(csv_row: str) -> int:
    """this function gets the CSV row of obu and returns id of rsu that sent this DM message"""
    # TODO Correct this later
    # return int(csv_row.split(",")[0])
    return int(csv_row[0])


def get_rsu_position_in_xy(csv_row: str) -> Tuple[int, int]:
    """this function return position of rsu"""
    # todo : complete this function later
    return 0, 0


def get_id(csv_row: str, is_rsu: bool = True) -> str:
    """this function return uniq id of csv row"""
    # TODO: change this later
    timestamp = get_timestamp(csv_row, is_rsu)
    crp_id = csv_row[0]
    beacon_id = csv_row[1]
    # station_id = get_station_id(pkt) # todo : add this variables to id
    # generation_time = get_generationdeltatime(pkt, False)
    # lat = get_latitude(pkt)
    # pkt_id = str(generation_time) + "_" + str(timestamp) + "_" + str(station_id) + "_" + str(lat)
    id = ",".join([str(timestamp), str(crp_id), str(beacon_id)])
    return id
