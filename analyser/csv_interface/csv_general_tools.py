"""this module contain some useful function for working with csv files """
from pathlib import Path
import csv
from typing import List
from config_AVNV import Conf



def read_csv_file(csv_file_path: Path) -> List:
    """
    Read a CSV file and return a list of values.

    Parameters:
    - csv_file_path (Path): The path to the CSV file.

    Returns:
    - list: A list of values read from the CSV file.
    """
    values = []
    with open(Path(Conf.csv_files_directory) / csv_file_path, 'r', newline='') as csvfile:
        csv_reader = csv.reader(csvfile)
        for row in csv_reader:
            # TODO: correct this later
            # values.append(','.join(row))
            values.append(row)

    return values
