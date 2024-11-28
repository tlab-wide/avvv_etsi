# Troubleshooting

## Build Troubleshooting

- In case you run into a SetupToolsDeprecationWarning issue when building any of the ROS packages, you need to upgrade your `setuptools` Python package so it lets you off with a warning:
```
pip3 install -U setuptools
```
To make sure you have it updated, execute the following:
```
pip3 show setuptools
```

## Run Troubleshooting

Currently logging is not built into AVVV. So, you need to run the modules manually and act accordingly or [report an issue](support_guidelines).

### Analyser

If you've configured the Analyser module, yet it isn't providing the desired output, run it manually to check for any specific error messages. From AVVV's root directory run:

```
source lib/avvv_etsi_venv/bin/activate
python3 analyser/main.py main
deactivate
```

### Visualiser

If you've configured the Visualiser module correctly and encounter issues such as application crashing, output not showing up or incomplete visualisation, run it manually to check for any specific error messages. From AVVV's root directory run:

```
source visualiser/install/setup.bash
ros2 launch visually visualiser.launch.py
```

In another terminal, play your ROSBAG file:

```
source visualiser/install/setup.bash
ros2 bag play path/to/your/rosbag2_avvv.db3
```

### Reporter

If you've configured the Reporter module, yet it isn't demonstrating the output properly, run it manually to check for any specific error messages. From AVVV's root directory run:

```
source lib/avvv_etsi_venv/bin/activate
python3 analyser/main.py plot
deactivate
```
