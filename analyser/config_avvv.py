import os
from configparser import ConfigParser
import json


class Conf:
    """
    this is config class
    """
    #
    # ros2 section
    ros2_new_types: list
    ros2_files_directory: str
    ros2_files_path: list = []  # this variable is not in config file
    ros2_topics: list

    #
    # pcap files section
    pcap_files_directory: str
    pcap_obu_files: list = []
    pcap_rsu_files: list = []

    #
    # output section
    report_output_directory_address: str
    rosbag_output_directory_address: str
    ros2_output_file_name: str
    cpm_topic_name_syntax: str  # not yet completed
    cpmn_topic_name_syntax: str
    network_status_topic_name_syntax: str
    equivalent_topics: list
    network_status_time: int

    #
    # advance
    simulation: bool
    creating_txtFile_from_outputRos_file: bool
    graphs_style: str
    time_reporter: bool
    position_reporter: bool
    distance_reporters: bool
    showing_graphs: bool
    save_graphs_image: bool
    save_graphs_pdf: bool
    save_graphs_csv: bool
    save_graphs_pickle: bool
    position_reporter_grid_x_size: float
    position_reporter_grid_y_size: float
    position_reporter_grid_z_size: float
    csv_files: bool
    max_difference_time_for_equivalent_tf_message: float
    rsu_effective_distance: float
    online_graph_display_speed: float
    rsu_only_graphs: bool

    #
    # position_reporter_packet_loss
    packet_loss_color: str
    packet_color: str
    just_show_packet_loss: bool
    packet_loss_point_size: float
    packet_point_size: float

    #
    # position_delay reporter
    position_delay_packet_point_size: float

    #
    # csv files
    csv_topics: list
    cpm_format_perceived_objects_number: int

    def __init__(self, config_file_name: str = "conf.ini"):
        """

        :param config_file_name:
        :return:
        """

        # reading information from config file
        configur = ConfigParser()
        configur.read(os.path.dirname(__file__) + "/" + config_file_name)

        # reading information about pcap files
        Conf.pcap_files_directory = json.loads(configur.get('pcap', 'pcap_files_directory'))
        for file in os.listdir(Conf.pcap_files_directory):
            if "rsu" in file.lower():
                Conf.pcap_rsu_files.append(file)
            elif "obu" in file.lower():
                Conf.pcap_obu_files.append(file)

        # reading information about rosbags files
        Conf.ros2_files_directory = json.loads(configur.get('ros2', 'rosbag_files_directory'))
        Conf.ros2_topics = json.loads(configur.get('ros2', 'topics'))

        # reading information about output file
        Conf.report_output_directory_address = json.loads(configur.get('output', 'report_output_address'))
        Conf.rosbag_output_directory_address = json.loads(configur.get('output', 'rosbag_output_address'))
        Conf.ros2_output_file_name = json.loads(configur.get('output', 'file_name'))
        Conf.cpm_topic_name_syntax = json.loads(configur.get('output', 'cpm_topic_name_syntax'))
        Conf.cpmn_topic_name_syntax = json.loads(configur.get('output', 'cpmn_topic_name_syntax'))
        Conf.network_status_topic_name_syntax = json.loads(configur.get('output', 'network_status_topic_name_syntax'))
        Conf.equivalent_topics = json.loads(configur.get('output', 'topics_equivalent_to_ros2_topics'))
        Conf.network_status_time = json.loads(configur.get('output', 'network_status_time'))

        # reading information of advance in config file
        Conf.simulation = Conf.boolean(json.loads(configur.get("advance", "simulation")))
        Conf.graphs_style = json.loads(configur.get("advance", "graphs_style"))
        Conf.creating_txtFile_from_outputRos_file = Conf.boolean(json.loads(
            configur.get('advance', 'creating_txtFile_from_outputRos_file')))
        Conf.time_reporter = Conf.boolean(json.loads(configur.get("advance", "time_reporter")))
        Conf.distance_reporters = Conf.boolean(json.loads(configur.get("advance", "distance_reporters")))
        Conf.rsu_only_graphs = Conf.boolean(json.loads(configur.get("advance", "rsu_only_graphs")))
        Conf.save_graphs_image = Conf.boolean(json.loads(configur.get("advance", "save_graphs_image")))
        Conf.save_graphs_csv = Conf.boolean(json.loads(configur.get("advance", "save_graphs_csv")))
        Conf.save_graphs_pdf = Conf.boolean(json.loads(configur.get("advance", "save_graphs_pdf")))
        Conf.save_graphs_pickle = Conf.boolean(json.loads(configur.get("advance", "save_graphs_pickle")))
        Conf.showing_graphs = Conf.boolean(json.loads(configur.get("advance", "showing_graphs")))
        Conf.position_reporter_grid_x_size = json.loads(configur.get("advance", "position_reporter_grid_x_size"))
        Conf.position_reporter_grid_y_size = json.loads(configur.get("advance", "position_reporter_grid_y_size"))
        Conf.position_reporter_grid_z_size = json.loads(configur.get("advance", "position_reporter_grid_z_size"))
        Conf.position_reporter = Conf.boolean(json.loads(configur.get("advance", "position_reporter")))
        Conf.csv_files = Conf.boolean(json.loads(configur.get("advance", "csv_files")))
        Conf.rsu_effective_distance = json.loads(configur.get("advance", "rsu_effective_distance"))
        Conf.max_difference_time_for_equivalent_tf_message = json.loads(
            configur.get("advance", "max_difference_time_for_equivalent_tf_message"))
        Conf.online_graph_display_speed = json.loads(configur.get("advance", "online_graph_display_speed"))
        for path in os.listdir(Conf.ros2_files_directory):
            Conf.ros2_files_path.append(Conf.ros2_files_directory + "/" + path)


        # reading information of position_reporter_packet_loss section in config file
        Conf.packet_loss_color = json.loads(configur.get("position_reporter_packet_loss", "packet_loss_color"))
        Conf.packet_color = json.loads(configur.get("position_reporter_packet_loss", "packet_color"))
        Conf.just_show_packet_loss = Conf.boolean(json.loads(configur.get("position_reporter_packet_loss", "just_show_packet_loss")))
        Conf.packet_loss_point_size = json.loads(configur.get("position_reporter_packet_loss", "packet_loss_point_size"))
        Conf.packet_point_size = json.loads(configur.get("position_reporter_packet_loss", "packet_point_size"))


        # reading information of position_reporter_delay section in config file
        Conf.position_delay_packet_point_size = json.loads(configur.get("position_reporter_delay", "packet_size"))

        # csv section
        Conf.csv_topics = json.loads(configur.get("csv_files", "csv_topics"))
        Conf.cpm_format_perceived_objects_number = json.loads(configur.get("csv_files", "cpm_format_perceived_objects_number"))

    @staticmethod
    def boolean(value: str) -> bool:
        return value.lower() == "true"
