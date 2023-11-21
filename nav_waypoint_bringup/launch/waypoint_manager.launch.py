#!/usr/bin/env python3

# Copyright (c) 2023 Naoki Takahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros


def generate_launch_description():
    this_pkg_share_dir = get_package_share_directory('nav_waypoint_bringup')

    waypoint_frame_id_param = {
        'waypoint_frame_id': launch.substitutions.LaunchConfiguration('waypoint_frame_id')
    }
    waypoints_file_param = {
        'waypoints_file': launch.substitutions.LaunchConfiguration('waypoints_file')
    }
    route_file_param = {
        'route_file': launch.substitutions.LaunchConfiguration('route_file')
    }
    use_sim_time_param = {
        'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
    }
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'waypoint_frame_id',
            default_value='map'
        ),
        launch.actions.DeclareLaunchArgument(
            'waypoints_file',
            default_value=os.path.join(
                this_pkg_share_dir,
                'config',
                'waypoints.yaml'
            )
        ),
        launch.actions.DeclareLaunchArgument(
            'route_file',
            default_value=os.path.join(
                this_pkg_share_dir,
                'config',
                'route.yaml'
            )
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        launch.actions.GroupAction([
            launch.actions.TimerAction(
                period=1.0,
                actions=[
                    launch_ros.actions.Node(
                        package='nav_waypoint_server',
                        executable='nav_waypoint_server_node',
                        name='nav_waypoint_server',
                        output='screen',
                        parameters=[
                            waypoint_frame_id_param,
                            waypoints_file_param,
                            use_sim_time_param
                        ]
                    )
                ]
            ),
            launch_ros.actions.Node(
                package='nav_waypoint_server',
                executable='nav_route_server_node',
                name='nav_route_server',
                output='screen',
                parameters=[
                    waypoint_frame_id_param,
                    route_file_param,
                    use_sim_time_param
                ]
            ),
            launch_ros.actions.Node(
                package='nav_waypoint_conversion',
                executable='pose_stamped_to_nav_waypoint_node',
                name='pose_stamped_to_nav_waypoint',
                output='screen',
                parameters=[
                    use_sim_time_param
                ],
                remappings=[
                    ('~/input_pose', '/set_waypoint'),
                    ('~/output_waypoint', 'nav_waypoint_server/regist')
                ]
            ),
            launch_ros.actions.Node(
                package='nav_waypoint_conversion',
                executable='nav_waypoint_to_pose_stamped_node',
                name='nav_waypoint_to_pose_stamped',
                output='screen',
                parameters=[
                    use_sim_time_param
                ],
                remappings=[
                    ('~/input_waypoint', 'nav_route_server/waypoint'),
                    ('~/output_pose', '/goal_pose')
                ]
            )
        ])
    ])
