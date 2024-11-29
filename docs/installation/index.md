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

To setup Wireshark build, switch to superuser and execute the debian setup shell script (you might be prompted for password):
```
sudo -s
```

```
. tools/debian-setup.sh --install-deb-deps --install-test-deps
exit
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

Finally, deactivate the virtual environment and go back to root directory using:
```
deactivate
cd ..
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
cd ../../../visualiser
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

Lastly, navigate back to the root directory:
```
cd ..
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

Lastly, navigate back to the root directory:
```
cd ../..
```

### Build Troubleshoot

Consult the [troubleshooting page](../support/troubleshooting.md).
