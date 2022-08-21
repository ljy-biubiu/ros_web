from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import launch_ros.actions
import ament_index_python

import os

def generate_launch_description():
    return LaunchDescription([
        # # include another launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource('/home/ljy/ros2_ws/src/cti_data_viewer/cti_mapsample/launch/robot_config.launch.py')
        #     ),
        #cti_vehicle_config_file_path = os.environ['CTI_ROBOT_CONFIG_PATH'] + "/vehicle.yaml"

        launch_ros.actions.Node(
            package='cti_mapsample', 
            executable='cti_mapsample', 
            parameters=[
            {'startMappingCmd': ament_index_python.get_package_share_directory("localization") + '/scripts/startMapping'},
            {'stopMappingCmd': ament_index_python.get_package_share_directory("localization") + '/scripts/stopMapping'},
            # {'calibration_launch_path': ament_index_python.get_package_share_directory("calibration") + '/launch/'},
            {'CTI_TOPIC_CONFIG_FILE_PATH': ament_index_python.get_package_share_directory("cti_mapsample") + '/config/topic_state.yaml'},
            #   {'shutdown_nowCmd_path': os.environ['HOME'] + '/cti-launch/boot_sh/'},
            {'CALIBRATION_LIDAR_CONFIG_PATH' : ament_index_python.get_package_share_directory("cti_mapsample") + '/config/calibration_lidar_topic.yaml'}
            ],
            output='screen',
            respawn=True
        ),
    ])
    
