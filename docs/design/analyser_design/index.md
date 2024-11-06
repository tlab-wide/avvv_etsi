# Analyser Design

This section will explain the design and the concepts of the Analyser module. The input section describes the input of this module and their requirements. The output section will give a detailed explanation of the output this module generates. Lastly, The architecture section will dissect the Analyser's body and go through its functional components.

## Analyser Input

The Analyser module's input consists of two types of data for each RSU or OBU; a PCAP file having captured the sent or received packets, and a ROS2 bag file including each unit's local messages exchanged between local ROS2 nodes.

### PCAP Files

PCAP is an API used to capture network data packets. Many other programmes such as Wireshark and TCPDUMP capture network and create such files. Current AVVV is only compatible with the [ETSI TS 103 324 V2.1.1 (2023-06)](https://www.etsi.org/deliver/etsi_ts/103300_103399/103324/02.01.01_60/ts_103324v020101p.pdf) standard and the [CPM](../../protocols/cpm_protocol) protocol.

### ROS2 Bag Files

`ros2 bag` is a command line tool for recording data published on topics in your system. It accumulates the data passed on any number of topics and saves it in a database. You can then replay the data to reproduce the results of your tests and experiments. Recording topics is also a great way to share your work and allow others to recreate it. The output of this command is a bag file. We're going to record our experiments topics and use the recordings in our analysis. For more details see the [prepare-input](../../how_to_guides/preparing_input) page.

## Analyser Output

The output the analyser produces is a set of various information in form of different files for various purposes. Below is a list of them.

### Universal ROS2 Bag File

The main output of the Analyser is a single bag file containing all the predictions, movements and network status for all Units and all RSU-OBU pairs. This file is used later in the Visualiser module to visualise what has happened during the experiment. The bag file will contain the following topics:

- RSU:
    - `/RSU_#/tf`: RSU location (tf2_msgs/msg/TFMessage)
    - `/RSU_#/detected_objects`: RSU prediction data (autoware_auto_perception_msgs/msg/PredictedObjects)
    - `/RSU_#/cpm`: RSU prediction data broadcast in the CPM format (cpm_ros_msgs/msg/CPMMessage)
- OBU:
    - `/OBU_#/tf`: OBU location (tf2_msgs/msg/TFMessage)
    - `/OBU_#/detected_objects`: OBU prediction data (autoware_auto_perception_msgs/msg/PredictedObjects)
    - `/OBU_#/RSU_#/cpmn`: RSU prediction data broadcast in the format of CPM received by an OBU along with the instantaneous network status of the packet. (cpm_ros_msgs/msg/CPMN)
    - `/OBU_#/RSU_#/network_status`: Average network status over some period of time (typically 1 second) (cpm_ros_msgs/msg/NetworkStatus)

### Instantaneous Network Status

The analyser will provide the results of its computation of delay, jitter, packet loss and RSSI per packet. Such results are available as graphs, animations and images for all RSUs (averaged across all OBUs) and all OBU-RSU pairs.

### Average Network Status

The analyser will also provide the results of its computation of delay, jitter, packet loss and RSSI averaged based on positional grids or time slots. Such results are available in form of heatmap files, graphs and images for all RSUs (averaged across all OBUs) and all OBU-RSU pairs.

## Architecture and Inner Workings


