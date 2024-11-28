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
