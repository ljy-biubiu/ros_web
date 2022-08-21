from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import ament_index_python
import os

from launch.actions import ExecuteProcess
import launch_ros.actions

def generate_launch_description():
    os.chdir('/opt/ros/humble/share/ctiwww')
    os.system('pwd')
    return LaunchDescription([
        # include another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( ament_index_python.get_package_share_directory("cti_mapsample") +'/launch/cti_mapsample.launch.py')
           ),

	# # Launch bridge
    #     ExecuteProcess(
    #         cmd=['node', ament_index_python.get_package_share_directory("ctiwww") +'/www/bin/rosbridge.js'],
    #         output='screen'),
    
    #     #os.popen('ls*.py').readlines(),
    #     #Launch bridge
    #     ExecuteProcess(
    #        cmd=['node', ament_index_python.get_package_share_directory("ctiwww") +'/www/examples/index.js'],
    #        output='screen')

    ])
    
    
