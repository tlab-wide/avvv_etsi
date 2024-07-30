import os
import sys
from ros2_interface.tools import register_new_types
from general_tools import create_directory
from config_avvv import Conf


def main():
    # Read the conf.ini file
    Conf()

    # Create output directories
    create_directory(Conf.report_output_directory_address)
    create_directory(Conf.rosbag_output_directory_address)

    # Create output CPM topics ROSBAG file and reports from input PCAP and ROSBAG files
    cpm_merger()


if __name__ == "__main__":

    # Register custom ROS2 types
    register_new_types(
        os.path.join(
            os.environ.get("AVVV_ETSI_HOME"),
            "visualiser",
            "src",
            "cpm_ros_msgs",
            "msg"))

    if sys.argv[1] == "plot":
        from plotting import plotting_from_pickle

        file_addresses = sys.argv[2:]
        for file_address in file_addresses:
            plotting_from_pickle(file_address)

    if sys.argv[1] == "main":
        from ros2_pcap_merger import cpm_merger
        main()
