#!/usr/bin/env python

"""
Example to launch a actuator_servos_advertiser node.

.. seealso::
    https://index.ros.org/doc/ros2/Launch-system/
"""

from launch import LaunchDescription
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_auv = os.path.join(
        get_package_share_directory('px4_ros_com'),
        'config',
        'led.yaml',
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='actuator_servos_advertiser',
            output='screen',
            name=['auv_controller'],
            parameters=[
                config_auv,
            ]
        ),
    ])
