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

Use **Add Waypoint** (shortcut **W**) to place a point and set its heading. **SetGoal** remains a navigation command. Drag waypoint markers to reposition them.

In the **Waypoint Editor** panel, select a registered point to edit its pose (metres/radians), reach radius or resume requirement, then click **Apply Waypoint**. Use **Rename** or **Delete Waypoint** for the selected point. A referenced point must first be removed from the route.

Build the visit order with **Append Selected**, **Up**, **Down** and **Remove Visit**, then click **Apply Route**. Repeated visits are allowed. **Reload / Discard Edits** restores server values. Click **Save Waypoints and Route** to save applied changes to the configured file; RViz's own Save Config saves the display layout only. Operation results appear below the panel. Failed marker updates restore the committed pose and show the reason in the marker label.

Waypoint and route edits can be applied independently. If another edit conflicts, reload before retrying. Numeric fields accept scientific notation.

For a different server, enter its node name (for example `/robot1/nav_waypoint_server`) and click **Connect / Discard Edits**. Set the Add Waypoint tool's **Waypoint Topic** to the registration converter's input topic. These connection settings are saved with the RViz configuration.

In **Route Execution**, click **Start Route** to begin following the route, or **Resume Route** at a resume-required waypoint. **Reset to First Visit** returns to the start and awaits a separate start command; it does not stop the robot. The panel shows route progress, visit number, target waypoint and any blocking reason. Waypoint markers remain draggable during execution. Enter another route server name and click **Connect** to change its connection; this is saved with the RViz configuration. Both panels can also be added from RViz's **Panels → Add New Panel** menu.

The CLI is also available:

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
| `goal_topic` | `/goal_pose` | Navigation goal output topic. |
| `set_waypoint_topic` | `/set_waypoint` | Waypoint registration input topic. |
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
