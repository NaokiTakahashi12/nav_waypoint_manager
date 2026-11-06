# Copyright (c) 2026
# SPDX-License-Identifier: MIT

import os

from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_ros.utilities import evaluate_parameters, normalize_parameters


def test_scoped_include_preserves_delayed_server_arguments(monkeypatch, tmp_path):
    observed = {}
    namespaces = {}
    remappings = {}

    def fake_node(**kwargs):
        def record(context):
            from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
            remappings[kwargs.get("name", kwargs["executable"])] = {
                key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                for key, value in kwargs.get("remappings", [])}

            namespaces[kwargs.get("name", kwargs["executable"])] = kwargs.get(
                "namespace", context.launch_configurations.get("ros_namespace", "/"))
            observed[kwargs.get('name', kwargs['executable'])] = evaluate_parameters(
                context, normalize_parameters(kwargs.get('parameters', [])))
        return OpaqueFunction(function=record)

    monkeypatch.setattr(launch_ros.actions, 'Node', fake_node)
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'waypoint_manager.launch.py')
    arguments = {
        'waypoint_frame_id': 'warehouse',
        'waypoints_file': str(tmp_path / 'points.yaml'),
        'route_file': str(tmp_path / 'route.yaml'),
        'waypoint_route_configuration_save_file': str(tmp_path / 'saved.yaml'),
        'reach_position_threshold': '2.5',
        'max_transform_age': '3.0',
        'publish_unchanged_goals': 'false',
        'use_sim_time': 'true',
        'goal_topic': '/test_robot/goal_pose',
        'set_waypoint_topic': '/test_robot/set_waypoint',
    }
    service = LaunchService()
    service.include_launch_description(LaunchDescription([GroupAction([
        launch_ros.actions.PushRosNamespace("test_robot"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=arguments.items()),
    ])]))
    assert service.run() == 0
    server = {key: value for params in observed['nav_waypoint_server']
              for key, value in params.items()}
    assert server == {
        'waypoint_frame_id': 'warehouse',
        'waypoints_file': arguments['waypoints_file'],
        'route_file': arguments['route_file'],
        'waypoint_route_configuration_save_file':
            arguments['waypoint_route_configuration_save_file'],
        'default_goal_reached_radius': 2.5,
        'use_sim_time': True,
    }
    route = {key: value for params in observed['nav_route_server']
             for key, value in params.items()}
    assert route['waypoint_frame_id'] == 'warehouse'
    assert route['reach_position_threshold'] == 2.5
    assert route['max_transform_age'] == 3.0
    for name in ('pose_stamped_to_nav_waypoint', 'nav_waypoint_to_pose_stamped'):
        params = {key: value for entry in observed[name] for key, value in entry.items()}
        assert params['waypoint_frame_id'] == 'warehouse'
        assert params['use_sim_time'] is True
    assert observed['nav_waypoint_to_pose_stamped'][1]['publish_unchanged_goals'] is False

    assert namespaces["nav_waypoint_server"] == "/test_robot"
    assert namespaces["nav_route_server"] == "/test_robot"

    assert remappings["nav_waypoint_to_pose_stamped"]["~/output_pose"] == "/test_robot/goal_pose"
    assert remappings["pose_stamped_to_nav_waypoint"]["~/input_pose"] == "/test_robot/set_waypoint"
