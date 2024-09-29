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

# Base altitude for unknown altitude values in DM messages
BASE_ALTITUDE = 0.0

# Pointcloud filename
PCD_FILENAME = ""

# Lanelet filename
LANELET_FILENAME = ""

# RSU-OBU connection distance range
RSU_OBU_CON_DIST = 300.0

# Topics and their transmitting links
RSU_DETECTION_TOPICS = [
    "/RSU_1/object_info",
    "/RSU_2/object_info",
    "/RSU_3/object_info",
    "/RSU_4/object_info",
]

OBU_DETECTION_TOPICS = [
    "/OBU_1/RSU_1/object_infon",
    "/OBU_1/RSU_2/object_infon",
    "/OBU_1/RSU_3/object_infon",
    "/OBU_1/RSU_4/object_infon",
]

RSU_FREESPACE_TOPICS = [
    "/RSU_1/freespace_info",
    "/RSU_2/freespace_info",
    "/RSU_3/freespace_info",
    "/RSU_4/freespace_info",
]

OBU_FREESPACE_TOPICS = [
    "/OBU_1/RSU_1/freespace_infon",
    "/OBU_1/RSU_2/freespace_infon",
    "/OBU_1/RSU_3/freespace_infon",
    "/OBU_1/RSU_4/freespace_infon",
]

RSU_SIGNAL_TOPICS = [
    "/RSU_1/signal_info",
    "/RSU_2/signal_info",
    "/RSU_3/signal_info",
    "/RSU_4/signal_info",
]

OBU_SIGNAL_TOPICS = [
    "/OBU_1/RSU_1/signal_infon",
    "/OBU_1/RSU_2/signal_infon",
    "/OBU_1/RSU_3/signal_infon",
    "/OBU_1/RSU_4/signal_infon",
]

# OBU_TOPICS must match the order in OBU_LIST
OBU_TOPICS = [
    "/OBU_1/tf",
]

# Link endpoints
# In format "RSU_ID OBU_ID" or "RSU_ID CLOUD_ID OBU_ID"
LINK_TOPICS = [
    "/OBU_1/RSU_1/object_info/network_status",
    "/OBU_1/RSU_1/freespace_info/network_status",
    "/OBU_1/RSU_1/signal_info/network_status",
    "/OBU_1/RSU_2/object_info/network_status",
    "/OBU_1/RSU_2/freespace_info/network_status",
    "/OBU_1/RSU_2/signal_info/network_status",
    "/OBU_1/RSU_3/object_info/network_status",
    "/OBU_1/RSU_3/freespace_info/network_status",
    "/OBU_1/RSU_4/object_info/network_status",
    "/OBU_1/RSU_4/freespace_info/network_status",
    "/OBU_1/RSU_4/signal_info/network_status",
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
    "RSU_1 4248.971 73445.545 20 330",
    "RSU_2 4512.198 72932.515 20 330",
    "RSU_3 4856.297 73143.781 20 330",
    "RSU_4 5175.195 73128.254 20 330",
]

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
OBU_LIST = [
    "OBU_1 3818 73722 20 210",
]

# In format "ID easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
CLOUD_LIST = [
    "CLD_0 3813 73789 60 330",
]

# In format "TL_ID/PL_ID CRP_ID Beacon_ID_set | easting(MGRS|metres) northing(MGRS|metres) altitude(MGRS|metres) heading(degree)"
SIGNAL_LIST = [
    # Intersection 1
    "TL_0 790531 17 18 19 33 34 35 | 3843.5 73732.7 25 30",
    "PL_0 790531 16 32 | 3845.06 73729.5 23 120",
    "PL_1 790531 16 32 | 3840.07 73739.1 23 300",
    "TL_1 790531 17 18 19 33 34 35 | 3857.0 73743.2 25 210",
    "PL_2 790531 16 32 | 3856.3 73746.9 23 300",
    "PL_3 790531 16 32 | 3861.2 73737.9 23 120",
    "TL_2 790531 49 50 51 | 3853.6 73734.6 25 120",
    "PL_4 790531 48 | 3838 73741.5 23 30",
    "PL_5 790531 48 | 3849.18 73748.1 23 210",
    
    # Intersection 2
    "TL_3 786434 17 18 19 33 34 35 | 3780.5 73700.7 25 30",
    "PL_6 786434 16 32 | 3782.06 73697.5 23 120",
    "PL_7 786434 16 32 | 3777.07 73707.1 23 300",
    "TL_4 786434 17 18 19 33 34 35 | 3794.0 73711.2 25 210",
    "PL_8 786434 16 32 | 3793.3 73714.9 23 300",
    "PL_9 786434 16 32 | 3798.2 73705.9 23 120",
    "TL_5 786434 49 50 51 | 3790.6 73702.6 25 120",
    "PL_10 786434 48 | 3775 73709.5 23 30",
    "PL_11 786434 48 | 3786.18 73716.1 23 210",
    
    # Intersection 3
    "TL_6 786433 17 18 19 33 34 35 | 3720.5 73668.7 25 30",
    "PL_12 786433 16 32 | 3722.06 73665.5 23 120",
    "PL_13 786433 16 32 | 3717.07 73675.1 23 300",
    "TL_7 786433 17 18 19 33 34 35 | 3734.0 73679.2 25 210",
    "PL_14 786433 16 32 | 3733.3 73682.9 23 300",
    "PL_15 786433 16 32 | 3738.2 73673.9 23 120",
    "TL_8 786433 49 50 51 | 3730.6 73670.6 25 120",
    "PL_16 786433 48 | 3715 73677.5 23 30",
    "PL_17 786433 48 | 3726.18 73684.1 23 210",
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
    {'rsu_detection_topic': RSU_DETECTION_TOPICS},
    {'obu_detection_topic': OBU_DETECTION_TOPICS},
    {'rsu_freespace_topic': RSU_FREESPACE_TOPICS},
    {'obu_freespace_topic': OBU_FREESPACE_TOPICS},
    {'rsu_signal_topic': RSU_SIGNAL_TOPICS},
    {'obu_signal_topic': OBU_SIGNAL_TOPICS},
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
    {'cloud_list': CLOUD_LIST},
    {'signal_list': SIGNAL_LIST},
    {'rsu_obu_con_dist': RSU_OBU_CON_DIST},
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
3