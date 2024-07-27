"""
there are some necessary general functions in this module
"""
import shutil
import os
import math
from typing import List, Dict
import mgrs
import csv


def lat_long_to_mgrs(latitude, longitude):
    m = mgrs.MGRS()
    mgrs_coords = m.toMGRS(latitude, longitude, MGRSPrecision=5)
    return mgrs_coords


def creating_directory(dir_address: str) -> None:
    """
    this function creating directory with dir_name name
    :param dir_address:
    :return:
    """
    if os.path.exists(dir_address):
        shutil.rmtree(dir_address)
    os.makedirs(dir_address)


def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def merge_dicts(dicts: List[dict], overwrite: bool = True, important_keys: bool = True) -> dict:
    """
    this function merges input dictionaries into one dictionary
    :param important_keys: is key of dicts important
    :param dicts: input dictionaries
    :param overwrite: If there are the same keys in the dictionaries, overwrite or not
    :return: merged dictionary
    """

    def important_key() -> dict:
        pass

    def not_important_key() -> dict:
        """
        output dictionary keys are the ordered numbers from 1 to ...
        :return:
        """
        temp_dict = {}
        counter = 0
        for dic in dicts:
            for value in dic.values():
                counter += 1
                temp_dict[counter] = value

        return temp_dict

    def overwrite_dicts_values() -> dict:
        """
        you can use this function to merge a list of dictionaries into a new dictionary
        this function overwrites values of the same keys
        :return:
        """
        temp_dict = {}
        for dic in dicts:
            temp_dict.update(dic)
        return temp_dict

    merged_output_dict: dict  # output dictionary

    # if important_keys:
    #     merged_output_dict = important_key()

    if not important_keys:
        merged_output_dict = not_important_key()

    if overwrite:
        merged_output_dict = overwrite_dicts_values()

    return merged_output_dict


def extract_id_from_pcap_file_name(file_name) -> str:
    """
    this function return id section of pcap file name
    :param file_name:
    :return:
    """
    return file_name.split('_')[-1].replace(".csv", "")


def csv_from_dict(info_dict: dict, file_name: str) -> None:
    """
    this function creating csv file from dictionary
    :param file_name: name of csv file
    :param info_dict: input dictionary that have information
    :return:
    """
    # Open the file in write mode and create a CSV writer object
    with open(file_name, mode="w", newline="") as file:
        writer = csv.writer(file)

        # writing metadata in the first of csv file
        metadata_comments = info_dict["metadata"]
        for metadata_comment in metadata_comments:
            writer.writerow([metadata_comment])
        del info_dict["metadata"]

        # Write the header row using the keys of the dictionary
        writer.writerow(info_dict.keys())

        # Write the values of each key as a row in the CSV file
        for row in zip(*info_dict.values()):
            writer.writerow(row)


def csv_metadata_creator(title="title", x_label="X", y_label="Y", z_label="Z", grid=0, rsu_x=0, rsu_y=0) -> List[str]:
    """
    this method creates metadat for csv files
    :param title: title of plot
    :param grid: grid size of plot ( is it is 0 that means we have no griding in plot )
    :return: list of comments for first of csv file
    """
    return [
        "# This CSV file is generated automatically don't edit that",
        "# Created by AVNV-CPM module ",
        "# METADATA #",
        "## TITLE : " + str(title),
        "## X-Label : " + str(x_label),
        "## Y_Label : " + str(y_label),
        "## Z_Label : " + str(z_label),
        "## Grid_Size : " + str(grid),
        "## RSU X Position: " + str(rsu_x),
        "## RSU Y Position: " + str(rsu_y)
    ]
