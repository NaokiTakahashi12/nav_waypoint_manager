// Copyright (c) 2026 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef NAV_WAYPOINT_MSGS__VALIDATION_HPP_
#define NAV_WAYPOINT_MSGS__VALIDATION_HPP_

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include <nav_waypoint_msgs/msg/waypoint_route_configuration.hpp>

namespace nav_waypoint_msgs
{
// Update timestamps do not change the meaning of a waypoint.
inline bool sameWaypoint(const msg::Waypoint & a, const msg::Waypoint & b)
{
  return a.name == b.name && a.frame_id == b.frame_id &&
         a.pose == b.pose && a.properties == b.properties;
}

inline double positiveNumber(const std::string & value)
{
  std::size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result) || result <= 0.0) {
    throw std::invalid_argument("Expected a finite positive number: " + value);
  }
  return result;
}

// Decode known policy once at the data boundary; preserve unknown properties.
inline std::pair<double, bool> reachSettings(const msg::Waypoint & w, double fallback = 1.0)
{
  std::pair<double, bool> result{fallback, false};
  for (const auto & p : w.properties) {
    if (p.key == "goal_reached_radius") {result.first = positiveNumber(p.value);}
    if (p.key == "stop_with_resume") {result.second = p.value == "true";}
  }
  return result;
}

inline void validateWaypoint(const msg::Waypoint & w, const std::string & frame)
{
  if (w.name.empty()) {
    throw std::invalid_argument("Waypoint name must not be empty");
  }
  if (!w.frame_id.empty() && w.frame_id != frame) {
    throw std::invalid_argument("Waypoint frame mismatch: " + w.name);
  }
  const auto & p = w.pose.position;
  const auto & q = w.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
  {
    throw std::invalid_argument("Invalid pose or non-unit quaternion: " + w.name);
  }
  std::set<std::string> keys;
  for (const auto & property : w.properties) {
    if (property.key.empty() || !keys.insert(property.key).second) {
      throw std::invalid_argument("Empty or duplicate property key: " + w.name);
    }
    if (property.key == "goal_reached_radius") {
      positiveNumber(property.value);
    } else if (property.key == "stop_with_resume") {
      if (property.value != "true" && property.value != "false") {
        throw std::invalid_argument("stop_with_resume must be true or false: " + w.name);
      }
    }
  }
}

inline void validateWaypoints(const msg::Waypoints & ws, const std::string & frame)
{
  if (ws.header.frame_id != frame) {
    throw std::invalid_argument("Waypoint list frame mismatch");
  }
  std::set<std::string> names;
  for (const auto & w : ws.waypoints) {
    validateWaypoint(w, frame);
    if (!names.insert(w.name).second) {
      throw std::invalid_argument("Duplicate waypoint name: " + w.name);
    }
  }
}
// Resolve only route-assigned locations, retaining visit order and repeated visits.
// Consumers share this interpretation of the editable configuration.
inline std::vector<msg::Waypoint> resolveRouteWaypoints(
  const msg::WaypointRouteConfiguration & configuration, const std::string & frame)
{
  if (frame.empty()) {throw std::invalid_argument("Configuration frame must not be empty");}
  validateWaypoints(configuration.registered_waypoints, frame);
  if (configuration.route.header.frame_id != frame) {
    throw std::invalid_argument("Route frame mismatch");
  }
  std::unordered_map<std::string, const msg::Waypoint *> registered;
  for (const auto & waypoint : configuration.registered_waypoints.waypoints) {
    registered.emplace(waypoint.name, &waypoint);
  }
  std::vector<msg::Waypoint> route_waypoints;
  route_waypoints.reserve(configuration.route.route.size());
  for (const auto & name : configuration.route.route) {
    const auto found = registered.find(name);
    if (found == registered.end()) {
      throw std::invalid_argument("Unknown route waypoint: " + name);
    }
    route_waypoints.push_back(*found->second);
  }
  return route_waypoints;
}
}  // namespace nav_waypoint_msgs
#endif  // NAV_WAYPOINT_MSGS__VALIDATION_HPP_
