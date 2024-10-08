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
DEFAULT = 4

# The following variables determine how different network features get visualised
LINK_COLOUR_VALUE = DELAY
LINK_OPACITY_VALUE = JITTER
LINK_THICKNESS_VALUE = RSSI
LINK_PACKET_DENSITY_VALUE = PACKET_LOSS

DELAY_BEST = 0.0
DELAY_WORST = 1.0
JITTER_BEST = 0.0
JITTER_WORST = 1.0
RSSI_BEST = 255
RSSI_WORST = 0
PACKET_LOSS_BEST = 0.0
PACKET_LOSS_WORST = 1.0

# Base altitude for unknown altitude values in DM messages
BASE_ALTITUDE = 20.0

# Pointcloud filename
PCD_FILENAME = ""

# Lanelet filename
LANELET_FILENAME = ""

# RSU-OBU connection distance range
RSU_OBU_CON_DIST = 300.0

# Topics and their transmitting links
RSU_DETECTION_TOPICS = [
]

OBU_DETECTION_TOPICS = [
]

RSU_FREESPACE_TOPICS = [
]

OBU_FREESPACE_TOPICS = [
]

RSU_SIGNAL_TOPICS = [
]

OBU_SIGNAL_TOPICS = [
]

# OBU_TOPICS must match the order in OBU_LIST
OBU_TOPICS = [
]

# Link endpoints
# In format "RSU_ID OBU_ID" or "RSU_ID CLOUD_ID OBU_ID"
LINK_TOPICS = [
]

# Pointcloud and lanelet map offset (x, y, z in metres)
MAP_OFFSET = [0.0, 0.0, 0.0]

OBU_LIDAR = "/"

RSU_DETECTED_COLOUR = "red"
OBU_DETECTED_COLOUR = "cyan"

RSU_FREESPACE_COLOUR = "yellow"
OBU_FREESPACE_COLOUR = "purple"

FREESPACE_WIDTH = 8.0

# The position and orientation of the RSUs, OBUs, cloud servers and traffic lights
# Note: All IDs must be unique

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
RSU_LIST = [
]

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
OBU_LIST = [
]

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
CLOUD = ""

# In format "TL_ID/PL_ID CRP_ID Beacon_ID_set | easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
SIGNAL_LIST = [
]

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

VISUALLY_DM_PARAMETERS = [
    {'base_altitude': BASE_ALTITUDE},
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
    {'rsu_detection_topics': RSU_DETECTION_TOPICS},
    {'obu_detection_topics': OBU_DETECTION_TOPICS},
    {'rsu_freespace_topics': RSU_FREESPACE_TOPICS},
    {'obu_freespace_topics': OBU_FREESPACE_TOPICS},
    {'rsu_signal_topics': RSU_SIGNAL_TOPICS},
    {'obu_signal_topics': OBU_SIGNAL_TOPICS},
    {'obu_topics': OBU_TOPICS},
    {'target_rsu_id': TARGET_RSU_ID},
    {'target_obu_id': TARGET_OBU_ID},
    {'rsu_detected_colour': RSU_DETECTED_COLOUR},
    {'obu_detected_colour': OBU_DETECTED_COLOUR},
    {'rsu_freespace_colour': RSU_FREESPACE_COLOUR},
    {'obu_freespace_colour': OBU_FREESPACE_COLOUR},
    {'freespace_width': FREESPACE_WIDTH},
    {'link_topics': LINK_TOPICS},
    {'rsu_list': RSU_LIST},
    {'obu_list': OBU_LIST},
    {'cloud': CLOUD},
    {'signal_list': SIGNAL_LIST},
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

    pkg_visually_dm = get_package_share_directory('visually_dm')

    rviz2_config = PathJoinSubstitution(
        [pkg_visually_dm, 'rviz', 'rviz_conf.rviz'])

    launch_description = LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_config]
        ),
        Node(
            package='visually_dm',
            executable='main',
            name='main',
            parameters=VISUALLY_DM_PARAMETERS
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
