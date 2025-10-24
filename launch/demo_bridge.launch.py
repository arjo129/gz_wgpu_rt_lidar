#!/usr/bin/env python3
# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Gazebo with the demo world and ROS 2 bridge for sensors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for demo with ROS 2 bridge."""
    pkg_gz_wgpu_rt_lidar = FindPackageShare('gz_wgpu_rt_lidar')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Launch arguments
    world_file = LaunchConfiguration('world_file') 
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            pkg_gz_wgpu_rt_lidar,
            'examples',
            'demo.sdf'
        ]),
        description='Path to the SDF world file'
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': world_file
        }.items()
    )

    # ROS-GZ Bridge for depth image
    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/my_robot/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    # ROS-GZ Bridge for lidar points
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/my_robot/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_world_file_cmd,
        gz_sim,
        bridge_depth,
        bridge_lidar,
    ])
