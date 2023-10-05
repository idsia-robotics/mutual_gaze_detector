import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch.actions
import launch_ros.actions

def generate_launch_description():
  
    face_landmarks_config_path = os.path.join(get_package_share_directory('users_landmarks_tracking'), 'config/face_landmarks.yaml')
  
    return LaunchDescription([
    GroupAction([
    launch_ros.actions.SetParametersFromFile(face_landmarks_config_path),
    launch_ros.actions.Node(
        package='users_landmarks_tracking',
        executable='face_landmarks_node',
        name='face_landmarks_node',
        parameters = [],
        output='screen',),    
    ])
    ])