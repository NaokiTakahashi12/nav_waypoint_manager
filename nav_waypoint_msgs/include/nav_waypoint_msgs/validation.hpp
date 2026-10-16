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

#include <nav_waypoint_msgs/msg/waypoints.hpp>

namespace nav_waypoint_msgs
{
inline double positiveNumber(const std::string & value)
{
  std::size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result) || result <= 0.0) {
    throw std::invalid_argument("Expected a finite positive number: " + value);
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
}  // namespace nav_waypoint_msgs
#endif  // NAV_WAYPOINT_MSGS__VALIDATION_HPP_
