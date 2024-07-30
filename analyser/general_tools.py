"""
there are some necessary general functions in this module
"""
import shutil
import os
import math
from typing import List
import mgrs


def lat_long_to_mgrs(latitude, longitude):
    m = mgrs.MGRS()
    mgrs_coords = m.toMGRS(latitude,
                           longitude,
                           MGRSPrecision=5)
    return mgrs_coords


def create_directory(dir_address: str) -> None:
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


def merge_dicts(dicts: List[dict],
                overwrite: bool = True,
                important_keys: bool = True) -> dict:
    """
    this function merges input dictionaries into one dictionary
    :param important_keys: is key of dicts important
    :param dicts: input dictionaries
    :param overwrite: If there are the same keys in the dictionaries, overwrite or not
    :return: merged dictionary
    """

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

    merged_output_dict: dict  # Output dictionary

    if not important_keys:
        merged_output_dict = not_important_key()

    if overwrite:
        merged_output_dict = overwrite_dicts_values()

    return merged_output_dict
