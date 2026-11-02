# Copyright (c) 2026 Naoki Takahashi
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

"""Integration regressions using real nodes and TF, without a navigation backend."""

import copy
import os
from pathlib import Path
import subprocess
import tempfile
import resource
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_waypoint_msgs.msg import (
    WaypointRouteConfiguration, Property, Route, RouteProgress, Waypoint, WaypointGoal, Waypoints)
from nav_waypoint_msgs.srv import DeleteWaypoint, RenameWaypoint, SetRoute, UpsertWaypoint
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate, MarkerArray


class WorkflowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Keep regression goals completely separate from any running robot graph.
        os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 50)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        cls.storage = tempfile.TemporaryDirectory(prefix='nav_waypoint_test_')
        cls.root = Path(cls.storage.name)
        cls.processes = []
        cls.logs = []
        rclpy.init()
        cls.node = rclpy.create_node('workflow_test')
        cls.messages = {}
        cls.history = {'goals': [], 'poses': [], 'modifications': [], 'markers': []}
        cls.subscriptions = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, message_type, topic, profile in [
            ('waypoints', Waypoints, '/nav_waypoint_server/waypoints', qos),
            ('route', Route, '/nav_route_server/route', qos),
            ('progress', RouteProgress, '/nav_route_server/progress', qos),
            ('goals', WaypointGoal, '/nav_route_server/waypoint', 10),
            ('poses', PoseStamped, '/nav_waypoint_to_pose_stamped/output_pose', 10),
            ('modifications', Waypoint, '/nav_waypoint_server/modify', 10),
            ('markers', MarkerArray, '/nav_waypoint_visualization_marker/route_marker', qos),
        ]:
            def callback(msg, key=key):
                cls.messages[key] = msg
                if key in cls.history:
                    cls.history[key].append(msg.waypoint if key == 'goals' else msg)
            cls.subscriptions.append(cls.node.create_subscription(
                message_type, topic, callback, profile))
        cls.tf_pub = cls.node.create_publisher(TFMessage, '/tf_static', qos)
        cls.resume_pub = cls.node.create_publisher(Bool, '/nav_route_server/release_resume', 3)
        cls.pose_pub = cls.node.create_publisher(
            PoseStamped, '/pose_stamped_to_nav_waypoint/input_pose', 3)
        cls.feedback_pub = cls.node.create_publisher(
            InteractiveMarkerFeedback,
            '/nav_waypoint_interactive_marker/interactive_marker/feedback', 10)
        (cls.root / 'route.yaml').write_text('waypoint_manager:\n  route: []\n')
        (cls.root / 'waypoints.yaml').write_text('waypoint_manager:\n  waypoints: {}\n')
        cls.spawn('nav_waypoint_server', 'nav_waypoint_server_node',
                  waypoints_file=str(cls.root / 'waypoints.yaml'),
                  waypoint_route_configuration_save_file=str(
                      cls.root / 'saved_waypoint_route_configuration.yaml'),
                  route_file=str(cls.root / 'route.yaml'))
        cls.spawn('nav_waypoint_server', 'nav_route_server_node',
                  reach_observe_frequency=20.0, publish_goal_with_reach_observe=2)
        cls.spawn('nav_waypoint_conversion', 'pose_stamped_to_nav_waypoint_node')
        cls.spawn('nav_waypoint_conversion', 'nav_waypoint_to_pose_stamped_node',
                  remaps=['~/input_waypoint:=/nav_route_server/waypoint'])
        cls.spawn('nav_waypoint_visualization', 'nav_waypoint_visualization_marker_node',
                  name='nav_waypoint_visualization_marker')
        cls.spawn('nav_waypoint_visualization', 'nav_waypoint_interactive_marker_node')
        try:
            cls.wait(lambda: 'waypoints' in cls.messages and 'route' in cls.messages)
        except Exception:
            cls.tearDownClass()
            raise

    @classmethod
    def spawn(cls, package, executable, name=None, remaps=None, **params):
        log = (cls.root / f'{executable}_{len(cls.processes)}.log').open('w')
        cls.logs.append(log)
        command = [str(Path(get_package_prefix(package)) / 'lib' / package / executable),
                   '--ros-args']
        if name:
            command += ['-r', '__node:=' + name]
        for remap in remaps or []:
            command += ['-r', remap]
        params.setdefault('waypoint_frame_id', 'warehouse')
        for key, value in params.items():
            command += ['-p', key + ':=' + str(value)]
        process = subprocess.Popen(
            command, stdout=log, stderr=log,
            preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_CORE, (0, 0)))
        cls.processes.append(process)
        return process

    @classmethod
    def tearDownClass(cls):
        for process in cls.processes:
            process.terminate()
        for process in cls.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in cls.logs:
            log.close()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.storage.cleanup()

    @classmethod
    def spin(cls, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def wait(cls, condition, timeout=6):
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= end:
                logs = '\n'.join(p.read_text()[-1600:] for p in cls.root.glob('*.log'))
                raise AssertionError('Timed out waiting for ROS data\n' + logs)
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def call(cls, kind, topic, request=None):
        client = cls.node.create_client(kind, topic)
        try:
            available = client.wait_for_service(timeout_sec=8)
            if not available:
                logs = '\n'.join(p.read_text()[-1600:] for p in cls.root.glob('*.log'))
                raise AssertionError(topic + '\n' + logs)
            future = client.call_async(request or kind.Request())
            rclpy.spin_until_future_complete(cls.node, future, timeout_sec=5)
            assert future.done(), topic
            return future.result()
        finally:
            cls.node.destroy_client(client)

    @classmethod
    def tf(cls, x, child='base_link'):
        transform = TransformStamped()
        transform.header.frame_id = 'warehouse'
        transform.child_frame_id = child
        transform.header.stamp = cls.node.get_clock().now().to_msg()
        transform.transform.translation.x = float(x)
        transform.transform.rotation.w = 1.0
        cls.tf_pub.publish(TFMessage(transforms=[transform]))
        cls.spin(0.15)

    @classmethod
    def waypoint(cls, name, x, stop=False):
        w = Waypoint(name=name, frame_id='warehouse')
        w.pose.position.x = float(x)
        w.pose.orientation.w = 1.0
        w.properties = [Property(key='goal_reached_radius', value='0.5'),
                        Property(key='stop_with_resume', value=str(stop).lower())]
        response = cls.call(UpsertWaypoint, '/nav_waypoint_server/upsert',
                            UpsertWaypoint.Request(waypoint=w))
        assert response.success, response.message
        cls.wait(lambda: any(item.name == name and item.pose == w.pose
                             for item in cls.messages['waypoints'].waypoints))
        cls.spin(0.1)
        return w

    @classmethod
    def route(cls, names):
        route = Route(route=names)
        route.header.frame_id = 'warehouse'
        response = cls.call(SetRoute, '/nav_waypoint_server/set_route',
                            SetRoute.Request(route=route))
        assert response.success, response.message
        cls.wait(lambda: cls.messages['route'].route == names)
        cls.spin(0.15)

    def setUp(self):
        self.extra_process_start = len(self.processes)
        self.route([])
        self.tf(0)
        for items in self.history.values():
            items.clear()

    def tearDown(self):
        # Release DDS participants created only for an individual scenario.
        for process in self.processes[self.extra_process_start:]:
            process.terminate()
        for process in self.processes[self.extra_process_start:]:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def start(self):
        self.resume_pub.publish(Bool(data=True))
        self.spin(0.2)

    def test_edits_and_republish_preserve_progress(self):
        self.waypoint('edit_a', 5)
        self.waypoint('edit_b', 10)
        self.route(['edit_a', 'edit_b'])
        self.start()
        self.wait(lambda: self.history['goals'] and self.history['goals'][-1].name == 'edit_a')
        self.tf(5)
        self.wait(lambda: self.history['goals'][-1].name == 'edit_b')
        self.call(Empty, '/nav_waypoint_server/publish')
        self.waypoint('edit_b', 12)
        self.wait(lambda: self.history['goals'][-1].pose.position.x == 12)
        self.assertEqual(self.messages['progress'].route_index, 1)
        self.assertEqual(self.history['goals'][-1].name, 'edit_b')

    def test_waiting_reissues_goal_and_completion_stays_completed(self):
        self.waypoint('stop_a', 5, stop=True)
        self.waypoint('stop_b', 10)
        self.route(['stop_a', 'stop_b'])
        self.start()
        self.tf(5)
        self.wait(lambda: self.messages['progress'].state == 'waiting_resume')
        count = len(self.history['goals'])
        self.spin(0.35)
        self.assertGreater(len(self.history['goals']), count)
        self.assertEqual(self.history['goals'][-1].name, 'stop_a')
        self.tf(4)
        self.assertEqual(self.messages['progress'].state, 'waiting_resume')
        self.resume_pub.publish(Bool(data=True))
        self.spin(0.15)
        self.assertEqual(self.messages['progress'].route_index, 0)
        self.tf(5)
        self.wait(lambda: self.history['goals'][-1].name == 'stop_b')
        self.tf(10)
        self.wait(lambda: self.messages['progress'].state == 'completed')
        self.call(Empty, '/nav_waypoint_server/publish')
        count = len(self.history['goals'])
        self.spin(0.25)
        self.assertEqual(self.messages['progress'].state, 'completed')
        self.assertEqual(len(self.history['goals']), count)

    def test_start_inside_stop_requires_a_second_resume(self):
        self.waypoint('inside', 0, stop=True)
        self.route(['inside'])
        self.start()
        self.wait(lambda: self.messages['progress'].state == 'waiting_resume')
        self.assertEqual(self.history['goals'][-1].name, 'inside')
        self.resume_pub.publish(Bool(data=True))
        self.wait(lambda: self.messages['progress'].state == 'completed')

    def test_early_resume_does_not_release_a_future_stop(self):
        self.waypoint('early', 5, stop=True)
        self.route(['early'])
        self.start()
        self.resume_pub.publish(Bool(data=True))
        self.spin(0.15)
        self.tf(5)
        self.wait(lambda: self.messages['progress'].state == 'waiting_resume')
        self.spin(0.15)
        self.assertEqual(self.messages['progress'].route_index, 0)

    def test_same_route_and_unreferenced_rename_keep_running(self):
        self.waypoint('unchanged_route', 5)
        self.waypoint('rename_unreferenced', 8)
        self.route(['unchanged_route'])
        self.start()
        self.wait(lambda: self.messages['progress'].state == 'running')
        self.route(['unchanged_route'])
        self.assertEqual(self.messages['progress'].state, 'running')
        command = [str(Path(get_package_prefix('nav_waypoint_bringup')) /
                       'lib/nav_waypoint_bringup/waypoint_edit'),
                   'rename', 'rename_unreferenced', 'renamed_unreferenced']
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.wait(lambda: process.poll() is not None, timeout=12)
        stdout, stderr = process.communicate()
        self.assertEqual(process.returncode, 0, (stdout, stderr))
        self.spin(0.2)
        self.assertEqual(self.messages['progress'].state, 'running')

    def test_edit_waiting_goal_clears_previous_wait(self):
        self.waypoint('edited_wait', 0, stop=True)
        self.route(['edited_wait'])
        self.start()
        self.wait(lambda: self.messages['progress'].state == 'waiting_resume')
        self.waypoint('edited_wait', 5, stop=False)
        self.wait(lambda: self.messages['progress'].state == 'running')
        self.tf(5)
        self.wait(lambda: self.messages['progress'].state == 'completed')

    def test_identical_and_unrelated_updates_do_not_increase_goal_rate(self):
        self.waypoint('rate_goal', 5)
        self.route(['rate_goal'])
        self.start()
        self.wait(lambda: self.messages['progress'].state == 'running')
        self.history['goals'].clear()
        started = time.monotonic()
        for _ in range(20):
            self.call(Empty, '/nav_waypoint_server/publish')
            self.spin(0.015)
        elapsed = time.monotonic() - started
        # 20 Hz / interval 2 = at most 10 Hz, allowing boundary jitter.
        self.assertLessEqual(len(self.history['goals']), int(elapsed * 10) + 2)

    def test_default_radius_is_explicit_and_noop_upsert_does_not_publish(self):
        w = Waypoint(name='default_radius', frame_id='warehouse')
        w.pose.orientation.w = 1.0
        response = self.call(UpsertWaypoint, '/nav_waypoint_server/upsert',
                             UpsertWaypoint.Request(waypoint=w))
        self.assertTrue(response.success, response.message)
        self.wait(lambda: any(w.name == 'default_radius'
                              for w in self.messages['waypoints'].waypoints))
        stored = next(w for w in self.messages['waypoints'].waypoints
                      if w.name == 'default_radius')
        self.assertEqual(float(next(p.value for p in stored.properties
                                    if p.key == 'goal_reached_radius')), 1.0)
        stamp = self.messages['waypoints'].header.stamp
        w.stamp = self.node.get_clock().now().to_msg()
        self.call(UpsertWaypoint, '/nav_waypoint_server/upsert',
                  UpsertWaypoint.Request(waypoint=w))
        self.spin(0.2)
        self.assertEqual(self.messages['waypoints'].header.stamp, stamp)

    def test_rename_is_atomic_and_updates_all_references(self):
        self.waypoint('atomic_old', 5, stop=True)
        self.route(['atomic_old', 'atomic_old'])
        configurations = []
        subscription = self.node.create_subscription(
            WaypointRouteConfiguration, '/nav_waypoint_server/configuration',
            configurations.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            response = self.call(RenameWaypoint, '/nav_waypoint_server/rename',
                                 RenameWaypoint.Request(
                                     old_name='atomic_old', new_name='atomic_new'))
            self.assertTrue(response.success, response.message)
            self.wait(lambda: configurations and
                      configurations[-1].route.route == ['atomic_new', 'atomic_new'])
            for configuration in configurations:
                known = {w.name for w in configuration.registered_waypoints.waypoints}
                self.assertTrue(all(name in known for name in configuration.route.route))
            rejected = self.call(RenameWaypoint, '/nav_waypoint_server/rename',
                                 RenameWaypoint.Request(old_name='atomic_new', new_name=''))
            self.assertFalse(rejected.success)
        finally:
            self.node.destroy_subscription(subscription)

    def test_stale_dynamic_tf_does_not_advance_route(self):
        self.waypoint('stale_tf_goal', 0)
        self.route(['stale_tf_goal'])
        progress = []
        subscription = self.node.create_subscription(
            RouteProgress, '/stale_tf_route/progress', progress.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher = self.node.create_publisher(TFMessage, '/tf', 10)
        resume = self.node.create_publisher(Bool, '/stale_tf_route/release_resume', 3)
        try:
            self.spawn('nav_waypoint_server', 'nav_route_server_node', name='stale_tf_route',
                       robot_frame_id='stale_base',
                       reach_observe_frequency=20.0)
            self.wait(lambda: progress and progress[-1].state == 'startup_wait')
            transform = TransformStamped()
            transform.header.frame_id = 'warehouse'
            transform.child_frame_id = 'stale_base'
            transform.transform.rotation.w = 1.0
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.stamp.sec -= 60
            for _ in range(3):
                publisher.publish(TFMessage(transforms=[transform]))
                self.spin(0.1)
            resume.publish(Bool(data=True))
            self.wait(lambda: progress[-1].state == 'blocked' and
                      'timestamp' in progress[-1].reason)
            self.assertEqual(progress[-1].route_index, 0)
            transform.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(TFMessage(transforms=[transform]))
            self.wait(lambda: progress[-1].state == 'completed')
        finally:
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)
            self.node.destroy_publisher(resume)

    def test_optional_adapter_suppresses_identical_position_goals(self):
        poses = []
        subscription = self.node.create_subscription(
            PoseStamped, '/dedup_adapter/output_pose', poses.append, 10)
        publisher = self.node.create_publisher(WaypointGoal, '/dedup_adapter/input_waypoint', 10)
        try:
            self.spawn('nav_waypoint_conversion', 'nav_waypoint_to_pose_stamped_node',
                       name='dedup_adapter', publish_unchanged_goals=False)
            self.wait(lambda: publisher.get_subscription_count() > 0)
            w = Waypoint(name='dedup', frame_id='warehouse')
            w.pose.orientation.w = 1.0
            publisher.publish(WaypointGoal(goal_id='visit_1', waypoint=w))
            self.wait(lambda: len(poses) == 1)
            for _ in range(5):
                w.stamp = self.node.get_clock().now().to_msg()
                publisher.publish(WaypointGoal(goal_id='visit_1', waypoint=w))
                self.spin(0.05)
            self.assertEqual(len(poses), 1)
            publisher.publish(WaypointGoal(goal_id='visit_2', waypoint=w))
            self.wait(lambda: len(poses) == 2)
            w.pose.position.x = 1.0
            publisher.publish(WaypointGoal(goal_id='visit_1', waypoint=w))
            self.wait(lambda: len(poses) == 3)
            w.name = 'another_visit'
            publisher.publish(WaypointGoal(goal_id='visit_1', waypoint=w))
            self.wait(lambda: len(poses) == 4)
        finally:
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)

    def test_slow_save_keeps_one_snapshot_and_does_not_block_edits(self):
        self.waypoint('snapshot_old', 5)
        self.route(['snapshot_old'])
        output = self.root / 'saved_waypoint_route_configuration.yaml'
        if output.exists():
            output.unlink()
        os.mkfifo(output)
        client = self.node.create_client(Trigger, '/nav_waypoint_server/save')
        writer = None
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            future = client.call_async(Trigger.Request())
            self.spin(0.2)
            self.assertFalse(future.done())
            started = time.monotonic()
            self.waypoint('snapshot_new', 10)
            self.route(['snapshot_new'])
            self.call(Empty, '/nav_route_server/reset')
            self.assertLess(time.monotonic() - started, 2.0)
            writer = os.open(output, os.O_WRONLY | os.O_NONBLOCK)
            os.write(writer, b'waypoint_manager: {}\n')
            os.close(writer)
            writer = None
            self.wait(future.done)
            self.assertTrue(future.result().success, future.result().message)
            saved = yaml.safe_load(output.read_text())['waypoint_manager']
            self.assertEqual(saved['route'], ['snapshot_old'])
            self.assertIn('snapshot_old', saved['waypoints'])
            self.assertNotIn('snapshot_new', saved['waypoints'])
        finally:
            if writer is not None:
                os.close(writer)
            self.node.destroy_client(client)

    def test_planar_marker_edit_preserves_height_roll_and_pitch(self):
        import math
        roll, pitch, yaw = 0.3, -0.2, 0.5
        w = Waypoint(name='spatial_marker', frame_id='warehouse')
        w.pose.position.z = 2.0
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        q = w.pose.orientation
        q.x, q.y = sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy
        q.z, q.w = cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy
        self.assertTrue(self.call(UpsertWaypoint, '/nav_waypoint_server/upsert',
                                  UpsertWaypoint.Request(waypoint=w)).success)
        self.wait(lambda: any(w.name == 'spatial_marker'
                              for w in self.messages['waypoints'].waypoints))
        self.spin(0.15)
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = 'warehouse'
        feedback.client_id = 'workflow_test'
        feedback.marker_name = 'spatial_marker'
        feedback.event_type = feedback.MOUSE_UP
        feedback.pose.position.x = 3.0
        feedback.pose.orientation.z = math.sin(0.9 / 2)
        feedback.pose.orientation.w = math.cos(0.9 / 2)
        self.feedback_pub.publish(feedback)
        self.wait(lambda: any(w.name == 'spatial_marker' and w.pose.position.x == 3.0
                              for w in self.messages['waypoints'].waypoints))
        result = next(w for w in self.messages['waypoints'].waypoints
                      if w.name == 'spatial_marker')
        self.assertEqual(result.pose.position.z, 2.0)
        q = result.pose.orientation
        actual_roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                                 1 - 2 * (q.x * q.x + q.y * q.y))
        actual_pitch = math.asin(2 * (q.w * q.y - q.z * q.x))
        self.assertAlmostEqual(actual_roll, roll)
        self.assertAlmostEqual(actual_pitch, pitch)

    def test_reset_creates_new_goal_id_for_identical_target(self):
        self.waypoint('reset_goal', 5)
        self.route(['reset_goal'])
        commands = []
        subscription = self.node.create_subscription(
            WaypointGoal, '/nav_route_server/waypoint', commands.append, 10)
        try:
            self.start()
            self.wait(lambda: len(commands) >= 2)
            old_id = commands[-1].goal_id
            self.assertTrue(all(c.goal_id == old_id for c in commands))
            self.call(Empty, '/nav_route_server/reset')
            self.start()
            self.wait(lambda: commands[-1].goal_id != old_id)
            self.assertEqual(commands[-1].waypoint.pose, commands[0].waypoint.pose)
        finally:
            self.node.destroy_subscription(subscription)

    def test_manager_restart_needs_no_reference_handshake(self):
        path = self.root / 'restart_waypoint_route_configuration.yaml'
        path.write_text(yaml.safe_dump({'waypoint_manager': {
            'frame_id': 'warehouse', 'route': ['restart'], 'waypoints': {'restart': {
                'pose': {'position': {'x': 5, 'y': 0, 'z': 0},
                         'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}}}}}))
        manager = self.spawn('nav_waypoint_server', 'nav_waypoint_server_node',
                             name='restart_data', waypoints_file=str(path))
        self.spawn('nav_waypoint_server', 'nav_route_server_node', name='restart_executor',
                   remaps=['nav_waypoint_server/configuration:=/restart_data/configuration'],
                   reach_observe_frequency=20.0)
        progress = []
        subscription = self.node.create_subscription(
            RouteProgress, '/restart_executor/progress', progress.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        resume = self.node.create_publisher(Bool, '/restart_executor/release_resume', 3)
        try:
            self.wait(lambda: progress and progress[-1].state == 'startup_wait' and
                      'release_resume' in progress[-1].reason and
                      resume.get_subscription_count() > 0)
            manager.terminate()
            manager.wait(timeout=5)
            resume.publish(Bool(data=True))
            self.wait(lambda: progress[-1].state == 'running')
            self.spawn('nav_waypoint_server', 'nav_waypoint_server_node',
                       name='restart_data', waypoints_file=str(path))
            self.assertTrue(self.call(SetRoute, '/restart_data/set_route',
                                      SetRoute.Request(route=Route(route=[]))).success)
            self.wait(lambda: progress[-1].state == 'empty')
            self.assertTrue(self.call(DeleteWaypoint, '/restart_data/delete',
                                      DeleteWaypoint.Request(name='restart')).success)
        finally:
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(resume)

    def test_invalid_configuration_clears_display_and_blocks_executor(self):
        publisher = self.node.create_publisher(
            WaypointRouteConfiguration, '/fault_configuration',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.spawn('nav_waypoint_visualization', 'nav_waypoint_visualization_marker_node',
                   name='fault_display',
                   remaps=['nav_waypoint_server/configuration:=/fault_configuration'])
        self.spawn('nav_waypoint_server', 'nav_route_server_node', name='fault_executor',
                   remaps=['nav_waypoint_server/configuration:=/fault_configuration'],
                   reach_observe_frequency=20.0)
        markers, progress = [], []
        subscriptions = [self.node.create_subscription(
            MarkerArray, '/fault_display/route_marker', markers.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.node.create_subscription(
                RouteProgress, '/fault_executor/progress', progress.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        try:
            configuration = WaypointRouteConfiguration()
            configuration.registered_waypoints.header.frame_id = 'warehouse'
            configuration.route.header.frame_id = 'warehouse'
            for name in ['first', 'last']:
                w = Waypoint(name=name, frame_id='warehouse')
                w.pose.orientation.w = 1.0
                configuration.registered_waypoints.waypoints.append(w)
            configuration.route.route = ['first', 'last']
            publisher.publish(configuration)
            self.wait(lambda: markers and any(m.type == m.LINE_STRIP
                                              for m in markers[-1].markers))
            configuration.route.route = ['first', 'missing', 'last']
            publisher.publish(configuration)
            self.wait(lambda: progress and progress[-1].state == 'blocked' and
                      'missing' in progress[-1].reason)
            self.wait(lambda: all(m.action == m.DELETE for m in markers[-1].markers))
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)

    def test_registered_candidates_are_not_execution_targets(self):
        self.waypoint('unassigned_candidate', 0, stop=True)
        self.waypoint('assigned_target', 5, stop=True)
        self.route(['assigned_target', 'assigned_target'])
        self.start()
        self.wait(lambda: self.history['goals'])
        self.assertEqual(self.messages['progress'].state, 'running')
        self.waypoint('unassigned_candidate', 0.1, stop=False)
        self.assertEqual(self.messages['progress'].route_index, 0)
        self.assertEqual(self.messages['progress'].state, 'running')
        self.assertTrue(all(w.name == 'assigned_target' for w in self.history['goals']))
        self.tf(5)
        self.wait(lambda: self.messages['progress'].state == 'waiting_resume')
        self.resume_pub.publish(Bool(data=True))
        self.wait(lambda: self.messages['progress'].route_index == 1 and
                  self.messages['progress'].state == 'waiting_resume')
        self.resume_pub.publish(Bool(data=True))
        self.wait(lambda: self.messages['progress'].state == 'completed')
        self.assertTrue(self.call(Trigger, '/nav_waypoint_server/save').success)
        saved = yaml.safe_load(
            (self.root / 'saved_waypoint_route_configuration.yaml').read_text()
        )['waypoint_manager']
        self.assertIn('unassigned_candidate', saved['waypoints'])
        self.assertEqual(saved['route'], ['assigned_target', 'assigned_target'])
        self.assertFalse(any(m.text == 'unassigned_candidate'
                             for m in self.messages['markers'].markers))

    def test_route_edit_notifies_configuration_without_republishing_registry(self):
        self.waypoint('configuration_target', 5)
        registry_stamp = self.messages['waypoints'].header.stamp
        self.route(['configuration_target'])
        self.assertEqual(self.messages['waypoints'].header.stamp, registry_stamp)
        self.route([])
        self.assertEqual(self.messages['waypoints'].header.stamp, registry_stamp)

    def test_invalid_edits_and_routes_are_rejected(self):
        valid = self.waypoint('valid', 5)
        self.route(['valid'])
        for value in ['abc', '-1', 'nan', '1garbage', '0']:
            invalid = copy.deepcopy(valid)
            invalid.properties[0].value = value
            response = self.call(UpsertWaypoint, '/nav_waypoint_server/upsert',
                                 UpsertWaypoint.Request(waypoint=invalid))
            self.assertFalse(response.success, value)
        invalid = copy.deepcopy(valid)
        invalid.frame_id = 'map'
        self.assertFalse(self.call(UpsertWaypoint, '/nav_waypoint_server/upsert',
                                   UpsertWaypoint.Request(waypoint=invalid)).success)
        invalid = copy.deepcopy(valid)
        invalid.pose.orientation.w = 0.0
        self.assertFalse(self.call(UpsertWaypoint, '/nav_waypoint_server/upsert',
                                   UpsertWaypoint.Request(waypoint=invalid)).success)
        response = self.call(SetRoute, '/nav_waypoint_server/set_route',
                             SetRoute.Request(route=Route(route=['missing'])))
        self.assertFalse(response.success)
        self.assertEqual(self.messages['route'].route, ['valid'])

    def test_save_reports_failure_and_roundtrips_frame_and_properties(self):
        self.waypoint('saved', 5, stop=True)
        self.route(['saved'])
        for topic in ['/nav_waypoint_server/save']:
            self.assertTrue(self.call(Trigger, topic).success)
        waypoints = yaml.safe_load(
            (self.root / 'saved_waypoint_route_configuration.yaml').read_text())
        route = yaml.safe_load((self.root / 'saved_waypoint_route_configuration.yaml').read_text())
        self.assertEqual(waypoints['waypoint_manager']['frame_id'], 'warehouse')
        self.assertEqual(waypoints['waypoint_manager']['waypoints']['saved']['properties']
                         ['stop_with_resume'], True)
        self.assertEqual(route['waypoint_manager']['route'], ['saved'])
        self.spawn('nav_waypoint_server', 'nav_waypoint_server_node', name='save_failure',
                   waypoints_file=str(self.root / 'waypoints.yaml'),
                   waypoint_route_configuration_save_file=str(self.root / 'missing' / 'file.yaml'))
        self.assertFalse(self.call(Trigger, '/save_failure/save').success)
        self.assertFalse((self.root / 'missing' / 'file.yaml').exists())
        self.spawn('nav_waypoint_server', 'nav_waypoint_server_node', name='reloaded',
                   waypoints_file=str(self.root / 'saved_waypoint_route_configuration.yaml'))
        reloaded = []
        subscription = self.node.create_subscription(
            Waypoints, '/reloaded/waypoints', reloaded.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            self.wait(lambda: bool(reloaded))
            self.assertEqual(reloaded[-1].header.frame_id, 'warehouse')
            self.assertTrue(any(w.name == 'saved' for w in reloaded[-1].waypoints))
        finally:
            self.node.destroy_subscription(subscription)

    def test_delete_checks_references_and_empty_route_clears_markers(self):
        self.waypoint('delete', 5)
        self.route(['delete'])
        self.assertFalse(self.call(DeleteWaypoint, '/nav_waypoint_server/delete',
                                   DeleteWaypoint.Request(name='delete')).success)
        self.route([])
        self.assertTrue(self.call(DeleteWaypoint, '/nav_waypoint_server/delete',
                                  DeleteWaypoint.Request(name='delete')).success)
        self.wait(lambda: self.messages['markers'].markers and
                  all(m.action == m.DELETE for m in self.messages['markers'].markers))

    def test_resume_survives_missing_tf(self):
        self.waypoint('delayed', 5)
        self.route(['delayed'])
        self.spawn('nav_waypoint_server', 'nav_route_server_node', name='delayed_route',
                   robot_frame_id='delayed_base',
                   reach_observe_frequency=20.0)
        progress = []
        subscription = self.node.create_subscription(
            RouteProgress, '/delayed_route/progress', progress.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher = self.node.create_publisher(Bool, '/delayed_route/release_resume', 3)
        try:
            self.wait(lambda: progress and publisher.get_subscription_count() > 0)
            publisher.publish(Bool(data=True))
            self.wait(lambda: progress[-1].state == 'blocked' and
                      'delayed_base' in progress[-1].reason)
            self.tf(0, child='delayed_base')
            self.wait(lambda: progress[-1].state == 'running')
        finally:
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)

    def test_custom_frame_conversion_and_marker_mouse_up(self):
        self.waypoint('marker', 5)
        self.route(['marker'])
        self.start()
        self.wait(lambda: self.history['poses'])
        self.assertEqual(self.history['poses'][-1].header.frame_id, 'warehouse')
        updates = []
        subscription = self.node.create_subscription(
            InteractiveMarkerUpdate,
            '/nav_waypoint_interactive_marker/interactive_marker/update', updates.append, 10)
        try:
            self.waypoint('marker', 6)
            self.wait(lambda: any(any(m.name == 'marker' for m in u.markers) for u in updates))
            feedback = InteractiveMarkerFeedback()
            feedback.header.frame_id = 'warehouse'
            feedback.client_id = 'workflow_test'
            feedback.marker_name = 'marker'
            feedback.event_type = feedback.MOUSE_UP
            feedback.mouse_point_valid = False
            feedback.pose.position.x = 7.0
            feedback.pose.orientation.w = 1.0
            self.feedback_pub.publish(feedback)
            self.wait(lambda: any(
                w.name == 'marker' and w.pose.position.x == 7.0
                for w in self.messages['waypoints'].waypoints))
            self.wait(lambda: bool(self.history['modifications']))
            self.assertEqual(self.history['modifications'][-1].properties[0].value, '0.5')
            pose = PoseStamped()
            pose.header.frame_id = 'warehouse'
            pose.pose.position.x = 8.0
            pose.pose.orientation.w = 1.0
            converted = []
            converter_subscription = self.node.create_subscription(
                Waypoint, '/pose_stamped_to_nav_waypoint/output_waypoint', converted.append, 10)
            try:
                self.wait(lambda: self.pose_pub.get_subscription_count() > 0)
                self.pose_pub.publish(pose)
                self.wait(lambda: bool(converted))
                self.assertEqual(converted[-1].frame_id, 'warehouse')
            finally:
                self.node.destroy_subscription(converter_subscription)
        finally:
            self.node.destroy_subscription(subscription)

    def test_cli_edits_properties_and_renames_references(self):
        self.waypoint('cli_old', 5)
        self.route(['cli_old', 'cli_old'])
        tool = str(Path(get_package_prefix('nav_waypoint_bringup')) /
                   'lib/nav_waypoint_bringup/waypoint_edit')
        result = subprocess.run([tool, 'upsert', 'cli_old', '--property', 'stop_with_resume=true'],
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.wait(lambda: any(w.name == 'cli_old' and any(
            p.key == 'stop_with_resume' and p.value == 'true' for p in w.properties)
            for w in self.messages['waypoints'].waypoints))
        result = subprocess.run([tool, 'rename', 'cli_old', 'cli_new'],
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.wait(lambda: self.messages['route'].route == ['cli_new', 'cli_new'])
        self.wait(lambda: not any(w.name == 'cli_old'
                                  for w in self.messages['waypoints'].waypoints))
        renamed = next(w for w in self.messages['waypoints'].waypoints if w.name == 'cli_new')
        self.assertTrue(any(p.value == 'true' for p in renamed.properties))
        self.assertEqual(self.messages['progress'].state, 'startup_wait')

    def test_save_preserves_other_sections_and_invalid_existing_output(self):
        combined = self.root / 'combined.yaml'
        combined.write_text('waypoint_manager:\n  frame_id: warehouse\n'
                            '  route: [preserved]\n  waypoints: {}\n  note: keep\n')
        self.spawn('nav_waypoint_server', 'nav_waypoint_server_node', name='combined_save',
                   waypoints_file=str(self.root / 'waypoints.yaml'),
                   waypoint_route_configuration_save_file=str(combined))
        self.assertTrue(self.call(Trigger, '/combined_save/save').success)
        result = yaml.safe_load(combined.read_text())['waypoint_manager']
        self.assertEqual(result['route'], [])
        self.assertEqual(result['note'], 'keep')
        self.assertEqual(result['waypoints'], {})
        broken = self.root / 'broken_output.yaml'
        previous = 'waypoint_manager: [invalid'
        broken.write_text(previous)
        self.spawn('nav_waypoint_server', 'nav_waypoint_server_node', name='protected_save',
                   waypoints_file=str(self.root / 'waypoints.yaml'),
                   waypoint_route_configuration_save_file=str(broken))
        self.assertFalse(self.call(Trigger, '/protected_save/save').success)
        self.assertEqual(broken.read_text(), previous)
        self.assertFalse(list(self.root.glob('*.tmp.*')))

    def test_invalid_observation_parameters_are_rejected(self):
        for frequency in [0.0, -1.0, 1001.0, 1e-300]:
            process = self.spawn(
                'nav_waypoint_server', 'nav_route_server_node', name='invalid_frequency',
                reach_observe_frequency=frequency)
            process.wait(timeout=5)
            self.assertNotEqual(process.returncode, 0, frequency)

    def test_malformed_configuration_fails_with_diagnostic(self):
        path = self.root / 'invalid.yaml'
        path.write_text('waypoint_manager: [invalid')
        process = self.spawn('nav_waypoint_server', 'nav_waypoint_server_node',
                             name='invalid_configuration', waypoints_file=str(path))
        process.wait(timeout=5)
        self.assertNotEqual(process.returncode, 0)


if __name__ == '__main__':
    unittest.main()
