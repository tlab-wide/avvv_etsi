"""
this module reading rosbag2 messages
"""
import os
import csv_tools
from config_avvv import Conf
from rosbags.serde import deserialize_cdr
from rosbags.rosbag2 import Reader


def tf_type_reader(rosbag_folder_path: str) -> dict:
    """

    :param rosbag_folder_path:
    :return:
    """
    with Reader(rosbag_folder_path) as reader:

        tf_messages = {}

        for connection, timestamp, rawdata in reader.messages():

            if connection.topic != "/tf":
                continue

            try:
                msg = (deserialize_cdr(rawdata, connection.msgtype))

                key = float(str(msg.transforms[0].header.stamp.sec) + "." + str(
                    msg.transforms[0].header.stamp.nanosec)) * 1000000000

                tf_messages[key] = msg


            except:
                print("cannot read tf type ")

        print("tf_message_length: ", len(tf_messages))
        return tf_messages


def txt_creator_from_rosbag(rosbag_folder_path: str):
    with open(Conf.report_output_directory_address + '/ros2_output_messages.txt', 'w') as f:

        f.write("MESSAGES : \n\n\n")
        # create reader instance and open for reading
        with Reader(rosbag_folder_path) as reader:
            counter = 0
            min_time = -1
            max_time = -1
            for connection, timestamp, rawdata in reader.messages():
                try:
                    counter += 1
                    if min_time == -1 or timestamp < min_time:
                        min_time = timestamp

                    if max_time == -1 or timestamp > max_time:
                        max_time = timestamp

                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    # print(msg)
                    f.write(str(msg) + "\n\n")


                except:
                    f.write("cant read this type :===> " + str(connection.msgtype) + "\n\n")

        print("min= ", min_time)
        f.write("min= " + str(min_time) + "\n\n")
        print("max= ", max_time)
        f.write("max= " + str(max_time) + "\n\n")
        print("counter= ", counter)
        f.write("counter= " + str(counter) + "\n\n")


def csv_creator_from_rosbag(output_files_address: str, pcap_dict_input: dict, ros_dict_input: dict) -> None:
    """

    :param output_files_address:
    :param ros_dict_input:
    :param pcap_dict_input:
    :return:
    """
    def get_csv_folder_name(topic_name: str) -> str:
        """
        This function takes a topic and return the name of the folder where the corresponding csv file should be saved
        :param topic_name:
        :return:
        """
        if topic_name.endswith("cpm"):
            return "RSU_CPM"
        if topic_name.endswith("network_status"):
            return "OBU_RSU_NETSTAT"
        if topic_name.endswith("tf"):
            return "TF"
        if topic_name.endswith("cpmn"):
            return "OBU_RSU_CPMN"

    def get_csv_file_name(topic_name: str) -> str:
        """
        This function takes a topic and return the name of the csv file
        :param topic_name:
        :return:
        """
        return (topic_name.replace("/", "", 1)).replace("/", "_") + ".csv"


    os.mkdir(output_files_address + "/topics_csv_files")

    # merging two dictionaries to one dictionary
    msgs_dict = pcap_dict_input | ros_dict_input

    for topic in msgs_dict:
        if topic.split("/")[-1] not in Conf.csv_topics:
            continue

        folder_name = get_csv_folder_name(topic).replace(" ", "")
        csv_file_name = get_csv_file_name(topic).replace(" ", "")

        if not os.path.exists(output_files_address + "/topics_csv_files/"+folder_name):
            os.mkdir(output_files_address + "/topics_csv_files/"+folder_name)


        csv_tools.creating_csv_file(folder_name, csv_file_name, msgs_dict[topic])

