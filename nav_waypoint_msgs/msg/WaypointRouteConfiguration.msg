# Editable and persistable waypoint registry plus the configured visit order.
# Registration alone never makes a waypoint a robot movement target.
nav_waypoint_msgs/Waypoints registered_waypoints
# Only names listed here become movement targets, in order; repeats are allowed.
nav_waypoint_msgs/Route route
# Publisher instance and monotonically increasing snapshot version.
string source_id
uint64 revision
