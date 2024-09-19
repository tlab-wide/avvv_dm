"""This module contains some useful function for working with csv files """
from pathlib import Path

import pandas


def read_csv_file(csv_file_path: Path) -> pandas.DataFrame:
    """
    Read a CSV file and returns a pandas.DataFrame of its content

    Parameters:
    - csv_file_path (Path): The path to the CSV file

    Returns:
    - list: A list of values read from the CSV file
    """
    df = pandas.read_csv(csv_file_path, header=None, on_bad_lines="skip")

    return df
