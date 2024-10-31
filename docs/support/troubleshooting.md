# Troubleshooting

## Build Troubleshooting

- In case you run into a SetupToolsDeprecationWarning issue when building any of the ROS packages, you need to downgrade you `setuptools` Python package using:
```
pip3 install setuptools==58.2.0
```
To make sure you have the right version of setuptools installed, execute the following:
```
pip3 show setuptools
```
