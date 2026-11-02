# nav_waypoint_manager

Register and edit waypoints in RViz, configure their visit order, and save the configuration.
Only waypoints listed in the route become movement targets.

## Run

From your colcon workspace with ROS 2 Humble sourced:

```bash
colcon build --packages-up-to nav_waypoint_manager
source install/setup.bash
ros2 launch nav_waypoint_bringup waypoint_manager.launch.py use_rviz:=true
```

Connect your navigation system to `/goal_pose` (`geometry_msgs/msg/PoseStamped`) and provide TF from `map` to `base_link`. Match `waypoint_frame_id` to your map frame.

Use RViz **SetGoal** to register a waypoint, or drag its marker to edit it. Use the CLI to register named points and set their visit order:

```bash
ros2 run nav_waypoint_bringup waypoint_edit upsert entrance --position 1 0 0 --yaw 0
ros2 run nav_waypoint_bringup waypoint_edit upsert desk --position 4 2 0 \
  --property goal_reached_radius=0.25 --property stop_with_resume=true
ros2 run nav_waypoint_bringup waypoint_edit set-route entrance desk
ros2 run nav_waypoint_bringup waypoint_edit save
ros2 topic pub --once /nav_route_server/release_resume std_msgs/msg/Bool '{data: true}'
```

Send `release_resume=true` to start after launch or a route change, and again at each resume-required waypoint. To restart from the first point, call reset, then send the start command:

```bash
ros2 service call /nav_route_server/reset std_srvs/srv/Empty '{}'
```

Run `ros2 run nav_waypoint_bringup waypoint_edit --help` for rename, delete and other options. Position values are in metres; `--yaw` is in radians.

## Launch arguments

Pass arguments as `name:=value`.

| Argument | Default | Purpose |
| --- | --- | --- |
| `waypoint_frame_id` | `map` | Coordinate frame for waypoints. |
| `waypoints_file` | Bundled `config/waypoints.yaml` | Waypoint input file. |
| `route_file` | Bundled `config/route.yaml` | Visit-order input file. |
| `waypoint_route_configuration_save_file` | See below | Save both waypoints and route. |
| `reach_position_threshold` | `1.0` | Default radius in metres for points without an explicit radius. |
| `max_transform_age` | `1.0` | Maximum dynamic TF age in seconds. |
| `publish_unchanged_goals` | `true` | Republish the current goal periodically. |
| `use_rviz` | `false` | Start RViz with the supplied configuration. |
| `use_sim_time` | `false` | Use the ROS simulation clock. |

The default save file is `${XDG_DATA_HOME:-$HOME/.local/share}/nav_waypoint_manager/waypoint_route_configuration.yaml`. Create the parent directory when using a custom save path.

## Configuration

Waypoints and route can share one YAML file:

```yaml
waypoint_manager:
  frame_id: map
  waypoints:
    entrance:
      pose:
        position: {x: 1.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      properties:
        goal_reached_radius: 0.25
        stop_with_resume: false
  route: [entrance]
```

Use a unit quaternion. Route entries must name registered waypoints; repeated visits are allowed. `goal_reached_radius` must be positive. `stop_with_resume: true` waits for a resume command while continuing to publish the same position goal; it does not send a robot stop command.

`waypoint_edit save` writes both waypoints and route to the configured save file. To load a saved configuration, set both input paths:

```bash
ros2 launch nav_waypoint_bringup waypoint_manager.launch.py \
  waypoints_file:=/path/to/waypoint_route_configuration.yaml \
  route_file:=/path/to/waypoint_route_configuration.yaml
```
