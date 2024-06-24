import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config = os.path.join(os.getcwd(), 'viz', 'one_config.rviz')
    print(rviz_config)
    drone_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('as2_viz'), 'launch'),
            '/as2_viz.launch.py']),
        launch_arguments={'rviz_config': rviz_config,
                          'namespace': 'cf0', 'color': 'green', 'use_sim_time': 'false',
                          'record_length': LaunchConfiguration('record_length')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('record_length', default_value='500',
                              description='Length for last poses.'),
        drone_0
    ])
