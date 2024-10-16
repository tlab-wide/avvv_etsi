# Installation

## Prerequisites

This is a source installation. It requires:
- OS: [Ubutnu 22.04](https://releases.ubuntu.com/22.04/)
- ROS: [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Git](https://git-scm.com/)

## Build

Take the steps as they're provided below in all sections:

Initially perform an update:
```
sudo apt update
```

Clone this repository:
```
git clone git@github.com:tlab-wide/avvv_etsi.git
```

Navigate to the root directory:
```
cd avvv_etsi
```

### Analyser

Navigate to `lib/wireshark`:
```
cd lib/wireshark
```

Setup Wireshark build:
```
sudo sh tools/debian-setup.sh
```

Create a new directory named `build` and navigate to it:
```
mkdir build && cd build
```

Configure and build Wireshark:
```
cmake -DBUILD_wireshark=OFF ..
make
```

Navigate back to `lib`:
```
cd ../..
```

Create a Python3 virtual environment:
```
python3 -m venv ./avvv_etsi_venv
```

Activate the virtual environment:
```
source avvv_etsi_venv/bin/activate
```

Navigate to `lib/pyshark/src`:
```
cd pyshark/src
```

Install `pyshark` locally:
```
python3 setup.py install
```

Navigate to the `analyser` directory:

```
cd ../../../analyser
```

Install the dependencies using pip:

```
pip3 install -r requirements.txt
```

### Visualiser

Build the geographic library. From the root directory of the repository, navigate to `lib/geographiclib-2.3`:

```
cd lib/geographiclib-2.3
```

Create a new directory for building:
```
mkdir build && cd build
```

Configure, build and install GeographicLib:

```
cmake ..
make
sudo make install
```

Navigate to the `visualiser` directory:

```
cd ../../visualiser
```

Resolve ROS2 dependencies using `rosdep`:

```
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Build the whole workspace using `colcon`. While still in the visualiser directory run:

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launcher

Install Qt6 libraries and dependencies:

```
sudo apt install qt6-tools-dev qt6-wayland
```

Navigate to the `ui` directory located in the root directory:

```
cd ui/visually_launcher
```

Build the application:

```
cmake -S . -B build
make -C build -j4
```
### Build Troubleshoot

- In case you run into a SetupToolsDeprecationWarning issue when building any of the ROS packages, you need to downgrade you `setuptools` Python package using:
```
pip3 install setuptools==58.2.0
```
To make sure you have the right version of setuptools installed, execute the following:
```
pip3 show setuptools
```

## Input Data

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

The `input_files` directory should be divided into 2 subdirectories called `pcap` and `rosbag` for comfort. The pcap files in `pcap` must be named after the name of their OBU or RSU IDs as in the figure above. `rosbag` must contain a directory named after their OBU or RSU IDs each containing a ROSBAG file corresponding to a PCAP file in `pcap`. There must exactly be one ROSBAG and PCAP file for each entity separately in `pcap` and `rosbag` and their names must match.

The PCAP files must contain data in the ETSI 2019 standard format. Get the standards from [here](https://www.etsi.org/deliver/etsi_tr/103500_103599/103562/02.01.01_60/tr_103562v020101p.pdf)

The input OBU ROSBAGs must contain the three essential topics:
- /tf: Locating the OBU or the RSU over time (tf2_msgs/msg/TFMessage)
- /perception/object_recognition/objects: The OBU's own predicted objects (autoware_auto_perception_msgs/msg/PredictedObjects)
- /v2x/cpm/objects: The predicted objects the OBU has received from an RSU (autoware_auto_perception_msgs/msg/PredictedObjects)

Likewise, the input RSU ROSBAGs must contain the following topics:
- /tf: Locating the OBU or the RSU over time (tf2_msgs/msg/TFMessage)
- /perception/object_recognition/objects: The RSU's own predicted objects (autoware_auto_perception_msgs/msg/PredictedObjects)

A sample input data is provided with the package that you can explore.

