import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


# Base altitude for unknown altitude values in DM messages
BASE_ALTITUDE = 20.0

# Pointcloud filename
PCD_FILENAME = ""

# Lanelet filename
LANELET_FILENAME = ""

# Topics and their transmitting links
RSU_DETECTION_TOPIC = "/object_info/dm [ RSU_0 OBU_0 | RSU_1 OBU_0 | RSU_2 OBU_0 | RSU_0 CLD_0 OBU_0 | RSU_1 CLD_0 OBU_0 | RSU_2 CLD_0 OBU_0 ]"

OBU_DETECTION_TOPIC = "/empty"

FREESPACE_TOPIC = "/freespace_info/dm [ RSU_0 CLD_0 OBU_0 | RSU_1 CLD_0 OBU_0 | RSU_2 CLD_0 OBU_0 ]"

SIGNAL_TOPIC = "/signal_info/local [ RSU_0 OBU_0 | RSU_1 OBU_0 | RSU_2 OBU_0 | RSU_0 CLD_0 OBU_0 | RSU_1 CLD_0 OBU_0 | RSU_2 CLD_0 OBU_0 ]"

# RSU_TOPICS must match the order in RSU_LIST
RSU_TOPICS = [
    "/rsu_0/tf"
]

# OBU_TOPICS must match the order in OBU_LIST
OBU_TOPICS = [
    "/obu_0/tf"
]

OBU_LIDAR = "/not_implemented"

RSU_DETECTED_COLOUR = "cyan"
OBU_DETECTED_COLOUR = "pink"

FREESPACE_COLOUR = "purple"

FREESPACE_WIDTH = 8.0

# The position and orientation of the RSUs, OBUs, cloud servers and traffic lights
# Note: All IDs must be unique

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
RSU_LIST = [
    "RSU_0 1000 1000 1000 0"
]

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
OBU_LIST = [
    "OBU_0 1000 1000 1000 0"
]

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
CLOUD_LIST = [
    "CLD_0 1000 1000 1000 0"
]

# In format "TL_ID/PL_ID CRP_ID Reference_ID_set | easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
SIGNAL_LIST = [
    # Intersection 1
    "TL_0 790531 17 18 19 20 49 50 51 52 | 0 -10 5 90", # Road 1
    "PL_0 790531 16 48 | -10 12 3 0", # Road 1
    "PL_1 790531 16 48 | 10 12 3 180", # Road 1
    "TL_1 790531 33 34 35 36 65 66 67 68 | 10 0 5 180", # Road 2
    "PL_2 790531 32 64 | -12 10 3 270", # Road 2
    "PL_3 790531 32 64 | -12 -10 3 90", # Road 2
    "TL_2 790531 17 18 19 20 49 50 51 52 | 0 10 5 270", # Road 3
    "PL_4 790531 48 16 | -10 -12 3 0", # Road 3
    "PL_5 790531 48 16 | 10 -12 3 180", # Road 3
    "TL_3 790531 33 34 35 36 65 66 67 68 | -10 0 5 0", # Road 4
    "PL_6 790531 64 32 | 12 10 3 270", # Road 4
    "PL_7 790531 64 32 | 12 -10 3 90", # Road 4
]

# Link endpoints
# In format "RSU_ID OBU_ID" or "RSU_ID CLOUD_ID OBU_ID"
LINKS = [
    "RSU_0 OBU_0",
    "RSU_1 OBU_0",
    "RSU_2 OBU_0",
    "RSU_0 CLD_0 OBU_0",
    "RSU_1 CLD_0 OBU_0",
    "RSU_2 CLD_0 OBU_0",
]

# RSU-OBU connection distance range
RSU_OBU_CON_DIST = 100.0

# Pointcloud and lanelet map offset (x, y, z in metres)
MAP_OFFSET = [0.0, 0.0, 0.0]

VISUALLY_DM_PARAMETERS = [
    {'base_altitude': BASE_ALTITUDE},
    {'pcd_file': PCD_FILENAME},
    {'rsu_detection_topic': RSU_DETECTION_TOPIC},
    {'obu_detection_topic': OBU_DETECTION_TOPIC},
    {'freespace_topic': FREESPACE_TOPIC},
    {'signal_topic': SIGNAL_TOPIC},
    {'rsu_topics': RSU_TOPICS},
    {'obu_topics': OBU_TOPICS},
    {'rsu_detected_colour': RSU_DETECTED_COLOUR},
    {'obu_detected_colour': OBU_DETECTED_COLOUR},
    {'freespace_colour': FREESPACE_COLOUR},
    {'freespace_width': FREESPACE_WIDTH},
    {'link_list': LINKS},
    {'rsu_list': RSU_LIST},
    {'obu_list': OBU_LIST},
    {'cloud_list': CLOUD_LIST},
    {'signal_list': SIGNAL_LIST},
    {'rsu_obu_con_dist': RSU_OBU_CON_DIST},
    {'pointcloud_offset': MAP_OFFSET}
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
3