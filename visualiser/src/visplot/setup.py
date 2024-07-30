from setuptools import setup

package_name = 'visplot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amirint',
    maintainer_email='amir.intelli@gmail.com',
    description='Visually extension for a real time statistics plotter subscribing on ROSBAG topics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "plotter_manager = visplot.plotter_manager:main",
        ],
    },
)
