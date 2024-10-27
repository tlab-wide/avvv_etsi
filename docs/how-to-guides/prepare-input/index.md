# Prepare input

This section will discuss the input data (data types, prototocols and application requirements regarding the data).

Basically, the analyser module takes in the ROSBAG and PCAP files and extracts the network information in form of reports and graphs and generates a single ROSBAG file for exhibition purposes.

Considering all of your input files are ready and gathered in a single folder called, for instance, `input_files`, the structure of the input data should be such as the following:

```
input_files
├── pcap
│   ├── OBU_0.pcap
│   ├── OBU_2.pcap
│   ├── RSU_0.pcap
│   ├── RSU_1.pcap
│   ...
└── rosbag
    ├── OBU_0
    │   ├── *.db3
    │   └── metadata.yaml
    ├── OBU_2
    │   ├── *.db3
    │   └── metadata.yaml
    ├── RSU_0
    │   ├── *.db3
    │   └── metadata.yaml
    ├── RSU_1
    │   ├── *.db3
    │   └── metadata.yaml
    ...
```

The `input_files` directory should be divided into 2 subdirectories called `pcap` and `rosbag` for comfort. The pcap files in `pcap` must be named after the name of their OBU or RSU IDs as in the snippet above. `rosbag` must contain a directory named after each OBU or RSU IDs each containing a ROSBAG file corresponding to a PCAP file in `pcap`. There must exactly be one ROSBAG and PCAP file for each entity separately in `pcap` and `rosbag` and their names must match.

The PCAP files must contain data in the ETSI 2019 standard format. Get the standards from [here](https://www.etsi.org/deliver/etsi_tr/103500_103599/103562/02.01.01_60/tr_103562v020101p.pdf)

The input OBU ROSBAGs must contain the three essential topics:
- /tf: Locating the OBU or the RSU over time (tf2_msgs/msg/TFMessage)
- /perception/object_recognition/objects: The OBU's own predicted objects (autoware_auto_perception_msgs/msg/PredictedObjects)
- /v2x/cpm/objects: The predicted objects the OBU has received from an RSU (autoware_auto_perception_msgs/msg/PredictedObjects)

Likewise, the input RSU ROSBAGs must contain the following topics:
- /tf: Locating the OBU or the RSU over time (tf2_msgs/msg/TFMessage)
- /perception/object_recognition/objects: The RSU's own predicted objects (autoware_auto_perception_msgs/msg/PredictedObjects)

A sample input data is provided with the package that you can explore.

