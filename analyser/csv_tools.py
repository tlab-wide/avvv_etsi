import csv
from rosbags.serde import deserialize_cdr
from rosbags.typesys.types import cpm_ros_msgs__msg__CPMMessage as cpm
from rosbags.typesys.types import cpm_ros_msgs__msg__CPMN as cpmn
from rosbags.typesys.types import cpm_ros_msgs__msg__NetworkStatus as netstat
from rosbags.typesys.types import tf2_msgs__msg__TFMessage as TF
from config_avvv import Conf
from typing import List


def get_msg_dicts(folder_name: str, msgs: list) -> list:
    """

    :return:
    """

    def get_msg_dict(folder: str, msg_content) -> dict:

        def get_netstat_msg_dict(netstat_msg: netstat) -> dict:
            return {
                "sec": netstat_msg.stamp.sec, "nanosec": netstat_msg.stamp.nanosec,
                "delay": netstat_msg.delay, "jitter": netstat_msg.jitter,
                "rssi": netstat_msg.rssi, "packet_loss": netstat_msg.packet_loss

            }

        def get_tf_msg_dict(tf_msg: TF) -> dict:

            return {
                "sec": tf_msg.transforms[0].header.stamp.sec,
                "nanosec": tf_msg.transforms[0].header.stamp.nanosec,
                "x_translation": tf_msg.transforms[0].transform.translation.x,
                "y_translation": tf_msg.transforms[0].transform.translation.y,
                "z_translation": tf_msg.transforms[0].transform.translation.z,
                "x_rotation": tf_msg.transforms[0].transform.rotation.x,
                "y_rotation": tf_msg.transforms[0].transform.rotation.y,
                "z_rotation": tf_msg.transforms[0].transform.rotation.z,
                "w_rotation": tf_msg.transforms[0].transform.rotation.w
            }

        def get_cpmn_msg_dict(cpmn_msg: cpmn) -> dict:

            fields_dic = {
                "sec": cpmn_msg.cpm.stamp.sec,
                "nanosec": cpmn_msg.cpm.stamp.nanosec,
                "latitude": cpmn_msg.cpm.cpm.cpm_parameters.management_container.reference_position.latitude,
                "longitude": cpmn_msg.cpm.cpm.cpm_parameters.management_container.reference_position.longitude,
                "delay": cpmn_msg.network_status.delay,
                "jitter": cpmn_msg.network_status.jitter,
                "rssi": cpmn_msg.network_status.rssi,
                "packet_loss": cpmn_msg.network_status.packet_loss
            }

            perceived_objects = [poc for poc in cpmn_msg.cpm.cpm.cpm_parameters.perceived_object_container.perceived_objects]

            index = 0
            for poc in perceived_objects:
                fields_dic["p_" + str(index) + "_object_id"] = poc.object_id
                fields_dic["p_" + str(index) + "_x_distance"] = poc.x_distance
                fields_dic["p_" + str(index) + "_y_distance"] = poc.y_distance
                fields_dic["p_" + str(index) + "_z_distance"] = poc.z_distance
                fields_dic["p_" + str(index) + "_x_speed"] = poc.x_speed
                fields_dic["p_" + str(index) + "_y_speed"] = poc.y_speed
                fields_dic["p_" + str(index) + "_z_speed"] = poc.z_speed
                fields_dic["p_" + str(index) + "_roll_angle"] = poc.roll_angle
                fields_dic["p_" + str(index) + "_pitch_angle"] = poc.pitch_angle
                fields_dic["p_" + str(index) + "_yaw_angle"] = poc.yaw_angle
                fields_dic["p_" + str(index) + "_planar_object_dimension1"] = poc.planar_object_dimension1
                fields_dic["p_" + str(index) + "_planar_object_dimension2"] = poc.planar_object_dimension2
                fields_dic["p_" + str(index) + "_vertical_object_dimension"] = poc.vertical_object_dimension

                index += 1

            return fields_dic

        def get_cpm_msg_dict(cpm_msg: cpm) -> dict:

            fields_dic = {
                "sec": cpm_msg.stamp.sec,
                "nanosec": cpm_msg.stamp.nanosec,
                "latitude": cpm_msg.cpm.cpm_parameters.management_container.reference_position.latitude,
                "longitude": cpm_msg.cpm.cpm_parameters.management_container.reference_position.longitude
            }

            perceived_objects = [poc for poc in cpm_msg.cpm.cpm_parameters.perceived_object_container.perceived_objects]

            index = 0
            for poc in perceived_objects:
                fields_dic["p_" + str(index) + "_object_id"] = poc.object_id
                fields_dic["p_" + str(index) + "_x_distance"] = poc.x_distance
                fields_dic["p_" + str(index) + "_y_distance"] = poc.y_distance
                fields_dic["p_" + str(index) + "_z_distance"] = poc.z_distance
                fields_dic["p_" + str(index) + "_x_speed"] = poc.x_speed
                fields_dic["p_" + str(index) + "_y_speed"] = poc.y_speed
                fields_dic["p_" + str(index) + "_z_speed"] = poc.z_speed
                fields_dic["p_" + str(index) + "_roll_angle"] = poc.roll_angle
                fields_dic["p_" + str(index) + "_pitch_angle"] = poc.pitch_angle
                fields_dic["p_" + str(index) + "_yaw_angle"] = poc.yaw_angle
                fields_dic["p_" + str(index) + "_planar_object_dimension1"] = poc.planar_object_dimension1
                fields_dic["p_" + str(index) + "_planar_object_dimension2"] = poc.planar_object_dimension2
                fields_dic["p_" + str(index) + "_vertical_object_dimension"] = poc.vertical_object_dimension

                index += 1

            return fields_dic

        if folder == "OBU_RSU_NETSTAT":
            return get_netstat_msg_dict(msg_content)
        if folder == "TF":
            return get_tf_msg_dict(msg_content)
        if folder == "OBU_RSU_CPMN":
            return get_cpmn_msg_dict(msg_content)
        if folder == "RSU_CPM":
            return get_cpm_msg_dict(msg_content)

    msg_dicts = []
    for msg in msgs:

        msg_content = msg[1]
        if folder_name == "TF":
            msg_content = deserialize_cdr(msg[2], msg[0].msgtype)

        msg_dicts.append(get_msg_dict(folder_name, msg_content))

    return msg_dicts


def get_msg_fields(folder_name: str) -> list:
    """

    :param folder_name:
    :return:
    """
    if folder_name == "OBU_RSU_NETSTAT":
        return ["sec", "nanosec", "delay", "jitter", "packet_loss", "rssi"]

    if folder_name == "TF":
        return ["sec", "nanosec", "x_translation", "y_translation", "z_translation",
                "x_rotation", "y_rotation", "z_rotation", "w_rotation"]

    if folder_name == "OBU_RSU_CPMN" or folder_name == "RSU_CPM":
        if folder_name == "OBU_RSU_CPMN":
            static_fields = ["sec", "nanosec", "latitude", "longitude", "delay", "jitter", "rssi", "packet_loss"]
        else:
            static_fields = ["sec", "nanosec", "latitude", "longitude"]

        dynamic_fields = ["object_id", "x_distance", "y_distance", "z_distance", "x_speed", "y_speed", "z_speed",
                          "roll_angle", "pitch_angle", "yaw_angle", "planar_object_dimension1",
                          "planar_object_dimension2", "vertical_object_dimension"]

        for index in range(Conf.cpm_format_perceived_objects_number):  # must be in config
            for field in dynamic_fields:
                static_fields.append("p_" + str(index) + "_" + field)

        return static_fields


def creating_csv_file(folder_name: str, csv_file_name: str, msgs: list) -> None:
    """

    :param folder_name:
    :param csv_file_name:
    :param msgs:
    :return:
    """
    # print(msgs)
    file_path = Conf.report_output_directory_address + "/topics_csv_files/" + folder_name + "/" + csv_file_name

    fields = get_msg_fields(folder_name)
    # print(msgs)
    msg_dicts = get_msg_dicts(folder_name, msgs)
    # print(msg_dicts)
    general_csv_creator(fields, msg_dicts, file_path)


def general_csv_creator(fields: list, msg_dicts: list, file_path: str) -> None:
    """

    :param fields:
    :param msg_dicts:
    :param file_path:
    :return:
    """
    # writing to csv file
    with open(file_path, 'w') as csvfile:
        # creating a csv dict writer object
        writer = csv.DictWriter(csvfile, fieldnames=fields)

        # writing headers (field names)
        writer.writeheader()

        # writing data rows
        writer.writerows(msg_dicts)


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
