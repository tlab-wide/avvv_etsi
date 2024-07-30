# AVNV cpm


## Table of contents

- Introduction
- Requirements
- Installation
- Configuration
- Troubleshooting



## Introduction

This project is part of the visualization project of sending CP messages between RSUs and OBUs, which is part of the research project on self-driving cars.
In this project, we merge the packets sent by the CPM protocol by the RSUs and received by the OBUs, as well as their rosbag2 files that have stored the information locally, and create an output rosbag2 file.
This output file is received and used by the visualization project

- For a full description of the [cpm protocol](https://www.etsi.org/deliver/etsi_tr/103500_103599/103562/02.01.01_60/tr_103562v020101p.pdf).
- Submit bug reports and feature suggestions, or track changes in the
  [issue queue](https://github.com/tlab-wide/avvv_etsi/issues).
  
## Requirements

This module requires the following modules:

- [rosbags](https://ternaris.gitlab.io/rosbags/)
- [pyshark](https://pypi.org/project/pyshark/)
- [numpy](https://numpy.org/)
- [matplotlib](https://matplotlib.org/)
- [configparser](https://pypi.org/project/configparser/)
- [json](https://pypi.org/project/jsons/)

## Configuration

To configure this project, you must use the conf.ini file that comes with the project
This file consists of 4 main parts: ros2, pcap, output, advance

1.ros2

A ) new_types: In this section, you must name the new ros2 types that are needed in the project to be registered in the project.
Note that the file of each type added in the above list must be placed in the cpm_ros_msgs/msg directory.
Also, the name of the new type must be the same as its file name

B ) files_directory: This is the directory that contains the input rosbag2 files

C ) topics: These are the topics we need from the rosbag2 files
In fact, we want to collect the messages with these topics from the input rosbag2 files and write them in our output file with our appropriate topics.

2.pcap

A ) rsu_files:This is a list of rsu filenames
These files contain packets sent by an rsu station
Note that the rsu files must be placed in the pcap directory

B ) obu_files:This is a list of obu filenames
These files contain packets received by an obu
Note that the obu files must be placed in the pcap directory

3.output

A ) file_name: output (rosbag2) file name 

B ) topics_equivalent_to_ros2_topics : This list, each item of which is a list with the names of two topics
Messages with the second topic in the rosbag 2 input files must be written in the rosbag 2 output file with the first topic.
It should be noted that the topics themselves are not used directly in the output and they become standard topics in the document.

4.advance

A ) reading_ros2_output_file : This variable can have true and false values
If this variable is true, after creating the output rosbag2 file, we immediately start reading it and save the output in the output directory with a txt file.
In fact, this work is a type of quick test of the output file

## Troubleshooting

If you see the error message: there are more than one rsu station id in rsu file:

	You should note that your rsu pcap file has more than one rsu station id, which is an important problem that you need to fix.
	If you are faced with this message, three options will open for you that you can use
		1. Ignore the problem and continue running the program
		2. Save the problem in a file and continue running the program (this part is not ready yet)
		3. Stop the program