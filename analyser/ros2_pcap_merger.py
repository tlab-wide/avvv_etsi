"""
This module is for merging PCAP and ROSBAG files into one ROSBAG file and creating some reports for the topics, files, and network status
"""
import topics
from nodes import NodesManager
from ros2_interface.ros2msg_gen import CPM
from ros2_interface.ros2file_read import csv_creator_from_rosbag, txt_creator_from_rosbag
from ros2_interface.ros2file_gen import create_rosbag2_file_from_pcapAndRos2_files
from config_avvv import Conf
from packet_interface import cpm_interface


def cpm_merger() -> None:
    """
    This is the primary function for merging PCAP files (CPM protocol packets)
    and ROS2 files into one ROSBAG file with CPM-related topics. This function
    also creates reports files for the files, topics, and network status
    :return:
    """

    # Save information of PCAP topics
    pcap_dict_information: dict = {}
    # Save information of ROSBAG topics
    ros_dict_information: dict

    # Read PCAP files and create RSU and OBU objects
    nodes_manager = NodesManager(Conf.pcap_obu_files, Conf.pcap_rsu_files)

    # Creating /rsu_#/cpm topics in final ROSBAG
    for rsu in nodes_manager.get_rsu_nodes():

        sender_cap = rsu.get_packets()

        rsu_cpm_dict: dict = topics.rsu_cpm(sender_cap)

        for topic in rsu_cpm_dict.keys():

            # Keep sender CPM messages for this topic
            rsu_cpm_msgs: list = []

            for pkt in rsu_cpm_dict[topic]:
                msg_info = [cpm_interface.get_epochtime(pkt), CPM(pkt).get_cpm()]
                rsu_cpm_msgs.append(msg_info)

            pcap_dict_information[topic] = rsu_cpm_msgs

    # Create /obu_#/rsu_#/network_status and /obu_#/rsu_#/cpmn topics in final ROSBAG
    for obu in nodes_manager.get_obu_nodes():
        for rsu in nodes_manager.get_rsu_nodes():

            sender_cap = rsu.get_packets()
            receiver_cap = obu.get_packets()

            obu_rsu_network_status_class = topics.obu_rsu_network_status(
                send_cap=sender_cap,
                receive_cap=receiver_cap,
                obu_id="0")

            if obu_rsu_network_status_class is not None:
                # Add network status object to RSU object
                rsu.append_network_status(obu_rsu_network_status_class)

                # Generate the /obu_#/rsu_#/network_status topic
                netstat_topic = obu_rsu_network_status_class.get_topic(Conf.network_status_topic_name_syntax)
                ros2type_network_status_list = obu_rsu_network_status_class.get_ros2type_network_status_list()
                pcap_dict_information[netstat_topic] = ros2type_network_status_list

                # Generate the /obu_#/rsu_#/cpmn topic
                cpmn_topic = obu_rsu_network_status_class.get_topic(Conf.cpmn_topic_name_syntax)
                ros2type_cpmn_list = obu_rsu_network_status_class.get_ros2type_cpmn_list()
                pcap_dict_information[cpmn_topic] = ros2type_cpmn_list

    # ROSBAG topics
    ros_dict_information = topics.collect_topics_from_rosbag2_file(
        Conf.ros2_files_path,
        Conf.ros2_topics,
        Conf.equivalent_topics)

    # Create output ROSBAG file
    create_rosbag2_file_from_pcapAndRos2_files(
        Conf.rosbag_output_directory_address,
        Conf.ros2_output_file_name,
        pcap_dict_information,
        ros_dict_information)

    # Create graphs of RSU
    if Conf.rsu_only_graphs:
        for rsu in nodes_manager.get_rsu_nodes():
            rsu.graphs()

    # Create CSV and TXT files from output ROSBAG file
    if Conf.creating_txtFile_from_outputRos_file:
        txt_creator_from_rosbag(
            f"{Conf.rosbag_output_directory_address}/rosbag2_{Conf.ros2_output_file_name}")

    if Conf.csv_files:
        csv_creator_from_rosbag(
            Conf.report_output_directory_address,
            pcap_dict_information,
            ros_dict_information)
