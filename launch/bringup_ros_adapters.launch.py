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


    return LaunchDescription([
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='auv_inner_pressure_adapter',
            output='screen',
            name=['auv_inner_pressure_adapter'],
        ),
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='vehicle_odometry_adapter',
            output='screen',
            name=['vehicle_odometry_adapter'],
        ),
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='water_temperature_adapter',
            output='screen',
            name=['water_temperature_adapter'],
        ),
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='actuator_motors_adapter',
            output='screen',
            name=['actuator_motors_adapter'],
        ),
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='vehicle_arm_toggle_adapter',
            output='screen',
            name=['vehicle_arm_toggle_adapter'],
        ),
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='actuator_servos_adapter',
            output='screen',
            name=['actuator_servos_adapter'],
        ),
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='esc_rpm_adapter',
            output='screen',
            name=['esc_rpm_adapter'],
        ),
        launch_ros.actions.Node(
            package='px4_ros_com',
            executable='sensors_adapter',
            output='screen',
            name=['sensors_adapter'],
        ),
    ])
