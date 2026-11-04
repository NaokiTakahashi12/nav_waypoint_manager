// Copyright (c) 2023 Naoki Takahashi
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

#include "nav_waypoint_server_node.hpp"
#include <chrono>
#include <random>

namespace nav_waypoint_server
{
NavWaypointServerNode::NavWaypointServerNode(const rclcpp::NodeOptions & options)
: Node("nav_waypoint_server", rclcpp::NodeOptions(options).use_intra_process_comms(false))
{
  configuration_.source_id = std::to_string(
    std::chrono::system_clock::now().time_since_epoch().count()) +
    "/" + std::to_string(std::random_device{}());
  param_listener_ = std::make_unique<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
  if (params_.waypoint_frame_id.empty() ||
    !std::isfinite(params_.default_goal_reached_radius) ||
    params_.default_goal_reached_radius <= 0.0)
  {
    throw std::invalid_argument("waypoint_frame_id must not be empty");
  }
  configuration_.registered_waypoints.header.frame_id = params_.waypoint_frame_id;
  if (fileExists(params_.waypoints_file)) {
    try {
      configuration_.registered_waypoints = loadWaypoints(params_.waypoints_file);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Cannot load waypoints: %s", e.what());
      throw;
    }
  } else {
    RCLCPP_WARN(get_logger(), "Starting with no waypoints: %s", params_.waypoints_file.c_str());
  }
  configuration_.route.header.frame_id = params_.waypoint_frame_id;
  const auto route_path =
    params_.route_file.empty() ? params_.waypoints_file : params_.route_file;
  if (fileExists(route_path)) {
    const auto root = loadConfigurationDocument(route_path, params_.waypoint_frame_id);
    const auto entries = root["waypoint_manager"]["route"];
    if (entries) {
      if (!entries.IsSequence()) {throw std::invalid_argument("route must be a sequence");}
      for (const auto & entry : entries) {
        configuration_.route.route.push_back(entry.as<std::string>());
      }
    }
  }
  validateConfiguredRoute(configuration_.route);
  configuration_publisher_ = create_publisher<nav_waypoint_msgs::msg::WaypointRouteConfiguration>(
    "~/configuration", rclcpp::QoS(1).transient_local());
  registered_waypoints_publisher_ = create_publisher<nav_waypoint_msgs::msg::Waypoints>(
    "~/waypoints", rclcpp::QoS(1).transient_local());
  regist_ = create_subscription<nav_waypoint_msgs::msg::Waypoint>(
    "~/regist", 5, [this](nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr w) {
      try {
        upsert(*w, false);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Rejected waypoint: %s", e.what());
      }
    });
  modify_ = create_subscription<nav_waypoint_msgs::msg::Waypoint>(
    "~/modify", 5, [this](nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr w) {
      try {
        upsert(*w, true);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Rejected modification: %s", e.what());
      }
    });
  set_route_service_ = create_service<nav_waypoint_msgs::srv::SetRoute>(
    "~/set_route", [this](nav_waypoint_msgs::srv::SetRoute::Request::ConstSharedPtr req,
    nav_waypoint_msgs::srv::SetRoute::Response::SharedPtr response) {
      try {
        if (req->expected_route.route != configuration_.route.route) {
          throw std::invalid_argument("Route changed; reload before applying");
        }
        validateConfiguredRoute(req->route);
        if (configuration_.route.route != req->route.route) {
          configuration_.route = req->route;
          configuration_.route.header.frame_id = params_.waypoint_frame_id;
          publishConfiguration();
        }
        response->configuration = configuration_;
        response->success = true;
        response->message = "Route accepted";
      } catch (const std::exception & e) {
        response->message = e.what();
      }
    });
  rename_service_ = create_service<nav_waypoint_msgs::srv::RenameWaypoint>(
    "~/rename", [this](nav_waypoint_msgs::srv::RenameWaypoint::Request::ConstSharedPtr req,
    nav_waypoint_msgs::srv::RenameWaypoint::Response::SharedPtr response) {
      auto found = std::find_if(
        configuration_.registered_waypoints.waypoints.begin(),
        configuration_.registered_waypoints.waypoints.end(),
        [&req](const nav_waypoint_msgs::msg::Waypoint & w) {return w.name == req->old_name;});
      if (found == configuration_.registered_waypoints.waypoints.end() || req->new_name.empty() ||
      std::any_of(
        configuration_.registered_waypoints.waypoints.begin(),
        configuration_.registered_waypoints.waypoints.end(),
        [&req](const nav_waypoint_msgs::msg::Waypoint & w) {return w.name == req->new_name;}))
      {
        response->message = "Old name must exist and new name must be nonempty and unused";
        return;
      }
      found->name = req->new_name;
      found->stamp = now();
      for (auto & name : configuration_.route.route) {
        if (name == req->old_name) {
          name = req->new_name;
        }
      }
      publishRegisteredWaypointsAndConfiguration();
      response->configuration = configuration_;
      response->success = true;
      response->message = "Waypoint and all route references renamed";
    });
  save_ = create_service<std_srvs::srv::Trigger>(
    "~/save", [this](rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
    std::shared_ptr<rmw_request_id_t> header, std_srvs::srv::Trigger::Request::SharedPtr) {
      const auto snapshot = configuration_;
      const auto frame = params_.waypoint_frame_id;
      const auto path = params_.waypoint_route_configuration_save_file.empty() ?
      params_.waypoints_file : params_.waypoint_route_configuration_save_file;
      auto reply = [service, header](bool success, const std::string & message) {
        std_srvs::srv::Trigger::Response response;
        response.success = success;
        response.message = message;
        try {
          service->send_response(*header, response);
        } catch (const std::exception &) {
        }
      };
      if (!saver_.submit(
        [snapshot, frame, path, reply]() {
          try {
            saveConfiguration(snapshot, path, frame);
            reply(true, "Saved configuration to " + path);
          } catch (const std::exception & e) {
            reply(false, e.what());
          }
        })) {reply(false, "Save queue is full; retry later");}
    });
  publish_ = create_service<std_srvs::srv::Empty>(
    "~/publish", [this](std_srvs::srv::Empty::Request::ConstSharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr) {publishRegisteredWaypointsAndConfiguration();});
  upsert_service_ = create_service<nav_waypoint_msgs::srv::UpsertWaypoint>(
    "~/upsert", [this](nav_waypoint_msgs::srv::UpsertWaypoint::Request::ConstSharedPtr request,
    nav_waypoint_msgs::srv::UpsertWaypoint::Response::SharedPtr response) {
      try {
        upsert(request->waypoint, false);
        response->success = true;
        response->message = "Waypoint registered or replaced";
      } catch (const std::exception & e) {
        response->message = e.what();
      }
    });
  update_service_ = create_service<nav_waypoint_msgs::srv::UpdateWaypoint>(
    "~/update", [this](nav_waypoint_msgs::srv::UpdateWaypoint::Request::ConstSharedPtr req,
    nav_waypoint_msgs::srv::UpdateWaypoint::Response::SharedPtr response) {
      try {
        const auto & points = configuration_.registered_waypoints.waypoints;
        const auto found = std::find_if(
          points.begin(), points.end(),
          [&req](const auto & w) {return w.name == req->expected_waypoint.name;});
        if (found == points.end() ||
        !nav_waypoint_msgs::sameWaypoint(*found, req->expected_waypoint))
        {
          throw std::invalid_argument("Waypoint changed or was deleted; reload before applying");
        }
        if (req->waypoint.name != found->name) {
          throw std::invalid_argument("Use rename to change a waypoint name");
        }
        upsert(req->waypoint, true);
        response->configuration = configuration_;
        response->success = true;
        response->message = "Waypoint updated";
      } catch (const std::exception & e) {
        response->message = e.what();
      }
    });
  update_pose_service_ = create_service<nav_waypoint_msgs::srv::UpdateWaypointPose>(
    "~/update_pose",
    [this](nav_waypoint_msgs::srv::UpdateWaypointPose::Request::ConstSharedPtr req,
    nav_waypoint_msgs::srv::UpdateWaypointPose::Response::SharedPtr response) {
      try {
        const auto found = std::find_if(
          configuration_.registered_waypoints.waypoints.begin(),
          configuration_.registered_waypoints.waypoints.end(),
          [&req](const nav_waypoint_msgs::msg::Waypoint & w) {return w.name == req->name;});
        if (found == configuration_.registered_waypoints.waypoints.end()) {
          throw std::invalid_argument("Unknown waypoint: " + req->name);
        }
        if (found->pose != req->expected_pose) {
          throw std::invalid_argument("Waypoint pose changed; retry from the current pose");
        }
        auto updated = *found;
        updated.frame_id = req->frame_id;
        updated.pose = req->pose;
        upsert(updated, true);
        response->success = true;
        response->message = "Waypoint pose updated";
      } catch (const std::exception & e) {
        response->message = e.what();
      }
    });
  delete_service_ = create_service<nav_waypoint_msgs::srv::DeleteWaypoint>(
    "~/delete", [this](nav_waypoint_msgs::srv::DeleteWaypoint::Request::ConstSharedPtr request,
    nav_waypoint_msgs::srv::DeleteWaypoint::Response::SharedPtr response) {
      if (std::find(
        configuration_.route.route.begin(), configuration_.route.route.end(),
        request->name) != configuration_.route.route.end())
      {
        response->message = "Remove the waypoint from the route before deleting it";
        return;
      }
      const auto found = std::find_if(
        configuration_.registered_waypoints.waypoints.begin(),
        configuration_.registered_waypoints.waypoints.end(),
        [&request](const nav_waypoint_msgs::msg::Waypoint & w) {return w.name == request->name;});
      if (found == configuration_.registered_waypoints.waypoints.end()) {
        response->message = "Unknown waypoint: " + request->name;
        return;
      }
      configuration_.registered_waypoints.waypoints.erase(found);
      publishRegisteredWaypointsAndConfiguration();
      response->configuration = configuration_;
      response->success = true;
      response->message = "Deleted waypoint";
    });
  publishRegisteredWaypointsAndConfiguration();
}

void NavWaypointServerNode::publishConfiguration()
{
  ++configuration_.revision;
  configuration_.route.header.stamp = now();
  configuration_publisher_->publish(configuration_);
}

void NavWaypointServerNode::publishRegisteredWaypointsAndConfiguration()
{
  configuration_.registered_waypoints.header.stamp = now();
  publishConfiguration();
  registered_waypoints_publisher_->publish(configuration_.registered_waypoints);
}

void NavWaypointServerNode::validateConfiguredRoute(const nav_waypoint_msgs::msg::Route & route)
const
{
  auto proposed_configuration = configuration_;
  proposed_configuration.route = route;
  if (proposed_configuration.route.header.frame_id.empty()) {
    proposed_configuration.route.header.frame_id = params_.waypoint_frame_id;
  }
  nav_waypoint_msgs::resolveRouteWaypoints(proposed_configuration, params_.waypoint_frame_id);
}

void NavWaypointServerNode::normalize(nav_waypoint_msgs::msg::Waypoint & w) const
{
  if (std::none_of(
      w.properties.begin(), w.properties.end(),
      [](const nav_waypoint_msgs::msg::Property & p) {return p.key == "goal_reached_radius";}))
  {
    nav_waypoint_msgs::msg::Property p;
    p.key = "goal_reached_radius";
    std::ostringstream value;
    value << std::setprecision(std::numeric_limits<double>::max_digits10) <<
      params_.default_goal_reached_radius;
    p.value = value.str();
    w.properties.push_back(p);
  }
  std::sort(
    w.properties.begin(), w.properties.end(),
    [](const nav_waypoint_msgs::msg::Property & a, const nav_waypoint_msgs::msg::Property & b) {
      return a.key < b.key;
    });
}

void NavWaypointServerNode::upsert(nav_waypoint_msgs::msg::Waypoint w, bool require_existing)
{
  nav_waypoint_msgs::validateWaypoint(w, params_.waypoint_frame_id);
  normalize(w);
  w.frame_id = params_.waypoint_frame_id;
  w.stamp = now();
  auto found = std::find_if(
    configuration_.registered_waypoints.waypoints.begin(),
    configuration_.registered_waypoints.waypoints.end(),
    [&w](const nav_waypoint_msgs::msg::Waypoint & current) {return current.name == w.name;});
  if (found == configuration_.registered_waypoints.waypoints.end()) {
    if (require_existing) {
      throw std::invalid_argument("Unknown waypoint: " + w.name);
    }
    configuration_.registered_waypoints.waypoints.push_back(w);
  } else {
    if (nav_waypoint_msgs::sameWaypoint(*found, w)) {return;}
    *found = w;
  }
  publishRegisteredWaypointsAndConfiguration();
}

nav_waypoint_msgs::msg::Waypoints NavWaypointServerNode::loadWaypoints(const std::string & path)
{
  const auto root = loadConfigurationDocument(path, params_.waypoint_frame_id);
  const auto entries = root["waypoint_manager"]["waypoints"];
  if (!entries || !entries.IsMap()) {
    throw std::invalid_argument("waypoints must be a mapping");
  }
  nav_waypoint_msgs::msg::Waypoints result;
  result.header.frame_id = params_.waypoint_frame_id;
  for (const auto & entry : entries) {
    nav_waypoint_msgs::msg::Waypoint w;
    w.name = entry.first.as<std::string>();
    w.frame_id = params_.waypoint_frame_id;
    const auto pose = entry.second["pose"];
    w.pose.position.x = pose["position"]["x"].as<double>();
    w.pose.position.y = pose["position"]["y"].as<double>();
    w.pose.position.z = pose["position"]["z"].as<double>();
    w.pose.orientation.x = pose["orientation"]["x"].as<double>();
    w.pose.orientation.y = pose["orientation"]["y"].as<double>();
    w.pose.orientation.z = pose["orientation"]["z"].as<double>();
    w.pose.orientation.w = pose["orientation"]["w"].as<double>();
    const auto properties = entry.second["properties"];
    if (properties && !properties.IsNull()) {
      if (!properties.IsMap()) {
        throw std::invalid_argument("properties must be a mapping: " + w.name);
      }
      for (const auto & property : properties) {
        nav_waypoint_msgs::msg::Property p;
        p.key = property.first.as<std::string>();
        p.value = property.second.as<std::string>();
        w.properties.push_back(p);
      }
    }
    normalize(w);
    result.waypoints.push_back(w);
  }
  nav_waypoint_msgs::validateWaypoints(result, params_.waypoint_frame_id);
  return result;
}

void NavWaypointServerNode::saveConfiguration(
  const nav_waypoint_msgs::msg::WaypointRouteConfiguration & snapshot,
  const std::string & path, const std::string & frame)
{
  nav_waypoint_msgs::resolveRouteWaypoints(snapshot, frame);
  auto root = prepareConfigurationDocumentForSave(path, frame);
  YAML::Node entries(YAML::NodeType::Map);
  for (const auto & w : snapshot.registered_waypoints.waypoints) {
    YAML::Node entry;
    entry["pose"]["position"]["x"] = w.pose.position.x;
    entry["pose"]["position"]["y"] = w.pose.position.y;
    entry["pose"]["position"]["z"] = w.pose.position.z;
    entry["pose"]["orientation"]["x"] = w.pose.orientation.x;
    entry["pose"]["orientation"]["y"] = w.pose.orientation.y;
    entry["pose"]["orientation"]["z"] = w.pose.orientation.z;
    entry["pose"]["orientation"]["w"] = w.pose.orientation.w;
    YAML::Node properties(YAML::NodeType::Map);
    for (const auto & property : w.properties) {
      properties[property.key] = property.value;
    }
    entry["properties"] = properties;
    entries[w.name] = entry;
  }
  root["waypoint_manager"]["waypoints"] = entries;
  YAML::Node names(YAML::NodeType::Sequence);
  for (const auto & name : snapshot.route.route) {
    names.push_back(name);
  }
  root["waypoint_manager"]["route"] = names;
  atomicSave(path, root);
}
}  // namespace nav_waypoint_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_server::NavWaypointServerNode)
