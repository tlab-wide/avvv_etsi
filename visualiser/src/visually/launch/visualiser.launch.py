import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


DELAY = 0
JITTER = 1
RSSI = 2
PACKET_LOSS = 3
NONE = 4

# The following variables determine how different network features get visualised
LINK_COLOUR_VALUE = NONE
LINK_OPACITY_VALUE = NONE
LINK_THICKNESS_VALUE = NONE
LINK_PACKET_DENSITY_VALUE = NONE

DELAY_BEST = 0.0
DELAY_WORST = 1.0
JITTER_BEST = 0.0
JITTER_WORST = 1.0
RSSI_BEST = 255
RSSI_WORST = 0
PACKET_LOSS_BEST = 0.0
PACKET_LOSS_WORST = 1.0

# Pointcloud filename
PCD_FILENAME = ""

# Lanelet filename
LANELET_FILENAME = ""

# Pointcloud and lanelet map offset (x, y, z in metres)
MAP_OFFSET = [0.0, 0.0, 0.0]

# Topics
TOPICS = []

# RSU-OBU connection distance range
RSU_OBU_CON_DIST = 300.0

# The target RSU and OBU for real time graphs and offline heatmaps
TARGET_RSU_ID = ""
TARGET_OBU_ID = ""

# Offline Heatmap
OFF_HM_PATH = ""
OFF_HM_ATTR = DELAY

# Real time Heatmap
ON_HM = False

# Real time graphs
RT_GRAPHS = False

VISUALLY_PARAMETERS = [
    {'pcd_file': PCD_FILENAME},
    {'map_offset': MAP_OFFSET},
    {'link_colour': LINK_COLOUR_VALUE},
    {'link_thickness': LINK_THICKNESS_VALUE},
    {'link_packet_density': LINK_PACKET_DENSITY_VALUE},
    {'link_opacity': LINK_OPACITY_VALUE},
    {'delay_best': DELAY_BEST},
    {'delay_worst': DELAY_WORST},
    {'jitter_best': JITTER_BEST},
    {'jitter_worst': JITTER_WORST},
    {'rssi_best': RSSI_BEST},
    {'rssi_worst': RSSI_WORST},
    {'packet_loss_best': PACKET_LOSS_BEST},
    {'packet_loss_worst': PACKET_LOSS_WORST},
    {'topics': TOPICS},
    {'target_rsu_id': TARGET_RSU_ID},
    {'target_obu_id': TARGET_OBU_ID},
    {'rsu_obu_con_dist': RSU_OBU_CON_DIST},
    {'off_hm_path': OFF_HM_PATH},
    {'off_hm_attr': OFF_HM_ATTR},
    {'on_hm': ON_HM},
]

VISPLOT_PARAMETERS = [
    {'target_rsu_id': TARGET_RSU_ID},
    {'target_obu_id': TARGET_OBU_ID}
]

def generate_launch_description():

    pkg_visually = get_package_share_directory('visually')

    rviz2_config = PathJoinSubstitution(
        [pkg_visually, 'rviz', 'rviz_conf.rviz'])

    launch_description = LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_config]
        ),
        Node(
            package='visually',
            executable='main',
            name='main',
            parameters=VISUALLY_PARAMETERS
        )
    ])

    if RT_GRAPHS:
        launch_description.add_action(
            Node(
                package='visplot',
                executable='plotter_manager',
                name='plotter_manager',
                parameters=VISPLOT_PARAMETERS
            )
        )

    if len(LANELET_FILENAME):
        launch_description.add_entity(
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    [os.path.join(
                    get_package_share_directory('map_loader'), 'launch'),
                    '/lanelet2_map_loader.launch.xml']
                ),
                launch_arguments={
                    'lanelet2_map_path': LANELET_FILENAME,
                    'x_offset': str(MAP_OFFSET[0]),
                    'y_offset': str(MAP_OFFSET[1]),
                    'z_offset': str(MAP_OFFSET[2]),
                }.items(),
            )
        )

    return launch_description
