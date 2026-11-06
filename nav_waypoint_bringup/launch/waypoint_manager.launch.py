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
    data_dir = os.path.join(
        os.environ.get('XDG_DATA_HOME', os.path.expanduser('~/.local/share')),
        'nav_waypoint_manager')
    os.makedirs(data_dir, exist_ok=True)
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
    waypoint_route_configuration_save_file_param = {
        'waypoint_route_configuration_save_file':
            launch.substitutions.LaunchConfiguration('waypoint_route_configuration_save_file')
    }
    use_sim_time_param = {
        'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
    }

    # Child exits during a parent shutdown must not request another shutdown.
    shutdown_event = launch.actions.OpaqueFunction(
        function=lambda context: [] if context.is_shutdown else [
            launch.actions.EmitEvent(event=launch.events.Shutdown())])

    def delayed_waypoint_server(context):
        # TimerAction runs after an enclosing launch group may have restored its
        # configuration. Capture the arguments while that scope is still active.
        argument_names = (
            'waypoint_frame_id', 'waypoints_file', 'route_file',
            'waypoint_route_configuration_save_file', 'reach_position_threshold',
            'use_sim_time',
        )
        captured_arguments = [
            launch.actions.SetLaunchConfiguration(
                name, launch.substitutions.LaunchConfiguration(name).perform(context))
            for name in argument_names
        ]
        return [launch.actions.TimerAction(
            period=1.0,
            actions=[launch.actions.GroupAction(captured_arguments + [
                launch_ros.actions.Node(
                    package='nav_waypoint_server',
                    executable='nav_waypoint_server_node',
                    name='nav_waypoint_server',
                    # The enclosing namespace scope also ends before the timer fires.
                    namespace=context.launch_configurations.get('ros_namespace', '/'),
                    output='screen',
                    parameters=[
                        waypoint_frame_id_param,
                        waypoints_file_param,
                        waypoint_route_configuration_save_file_param,
                        route_file_param,
                        {'default_goal_reached_radius':
                         launch_ros.parameter_descriptions.ParameterValue(
                             launch.substitutions.LaunchConfiguration(
                                 'reach_position_threshold'), value_type=float)},
                        use_sim_time_param
                    ],
                    on_exit=shutdown_event
                )
            ])]
        )]

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
            'waypoint_route_configuration_save_file',
            default_value=os.path.join(data_dir, 'waypoint_route_configuration.yaml')
        ),
        launch.actions.DeclareLaunchArgument('reach_position_threshold', default_value='1.0'),
        launch.actions.DeclareLaunchArgument('max_transform_age', default_value='1.0'),
        launch.actions.DeclareLaunchArgument('publish_unchanged_goals', default_value='true'),
        launch.actions.DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        launch.actions.DeclareLaunchArgument('set_waypoint_topic', default_value='/set_waypoint'),
        launch.actions.DeclareLaunchArgument('use_rviz', default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        launch.actions.GroupAction([
            launch.actions.OpaqueFunction(function=delayed_waypoint_server),
            launch_ros.actions.Node(
                package='nav_waypoint_server',
                executable='nav_route_server_node',
                name='nav_route_server',
                output='screen',
                parameters=[
                    waypoint_frame_id_param,
                    {'reach_position_threshold': launch_ros.parameter_descriptions.ParameterValue(
                        launch.substitutions.LaunchConfiguration('reach_position_threshold'),
                        value_type=float)},
                    {'max_transform_age': launch_ros.parameter_descriptions.ParameterValue(
                        launch.substitutions.LaunchConfiguration('max_transform_age'),
                        value_type=float)},
                    use_sim_time_param
                ],
                on_exit=shutdown_event
            ),
            launch_ros.actions.Node(
                package='nav_waypoint_visualization',
                executable='nav_waypoint_visualization_marker_node',
                name='nav_waypoint_visualization_marker',
                output='screen',
                parameters=[
                    use_sim_time_param
                ],
                on_exit=shutdown_event
            ),
            launch_ros.actions.Node(
                package='nav_waypoint_visualization',
                executable='nav_waypoint_interactive_marker_node',
                name='nav_waypoint_interactive_marker',
                output='screen',
                parameters=[
                    use_sim_time_param
                ],
                on_exit=shutdown_event
            ),
            launch_ros.actions.Node(
                package='nav_waypoint_conversion',
                executable='pose_stamped_to_nav_waypoint_node',
                name='pose_stamped_to_nav_waypoint',
                output='screen',
                parameters=[
                    waypoint_frame_id_param,
                    use_sim_time_param
                ],
                remappings=[
                    ('~/input_pose', launch.substitutions.LaunchConfiguration('set_waypoint_topic')),
                    ('~/output_waypoint', 'nav_waypoint_server/regist')
                ],
                on_exit=shutdown_event
            ),
            launch_ros.actions.Node(
                package='nav_waypoint_conversion',
                executable='nav_waypoint_to_pose_stamped_node',
                name='nav_waypoint_to_pose_stamped',
                output='screen',
                parameters=[
                    waypoint_frame_id_param,
                    {'publish_unchanged_goals': launch_ros.parameter_descriptions.ParameterValue(
                        launch.substitutions.LaunchConfiguration('publish_unchanged_goals'),
                        value_type=bool)},
                    use_sim_time_param
                ],
                remappings=[
                    ('~/input_waypoint', 'nav_route_server/waypoint'),
                    ('~/output_pose', launch.substitutions.LaunchConfiguration('goal_topic'))
                ],
                on_exit=shutdown_event
            ),
            launch_ros.actions.Node(
                package='rviz2', executable='rviz2',
                arguments=['-d', os.path.join(this_pkg_share_dir, 'rviz', 'waypoint.rviz'),
                           '-f', launch.substitutions.LaunchConfiguration('waypoint_frame_id')],
                parameters=[use_sim_time_param],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('use_rviz'))
            )
        ])
    ])
