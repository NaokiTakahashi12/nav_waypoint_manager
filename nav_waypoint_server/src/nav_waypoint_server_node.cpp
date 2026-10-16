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

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_waypoint_msgs/msg/route.hpp>
#include <nav_waypoint_msgs/srv/upsert_waypoint.hpp>
#include <nav_waypoint_msgs/srv/delete_waypoint.hpp>
#include <nav_waypoint_msgs/srv/reserve_route.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <nav_waypoint_server/nav_waypoint_server_node_params.hpp>

#include "dataset_io.hpp"
#include "async_saver.hpp"

namespace nav_waypoint_server
{
class NavWaypointServerNode : public rclcpp::Node
{
public:
  explicit NavWaypointServerNode(const rclcpp::NodeOptions & options)
  : Node("nav_waypoint_server", rclcpp::NodeOptions(options).use_intra_process_comms(false))
  {
    param_listener_ = std::make_unique<ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();
    if (params_.waypoint_frame_id.empty() ||
      !std::isfinite(params_.default_goal_reached_radius) ||
      params_.default_goal_reached_radius <= 0.0)
    {
      throw std::invalid_argument("waypoint_frame_id must not be empty");
    }
    waypoints_.header.frame_id = params_.waypoint_frame_id;
    if (fileExists(params_.waypoints_file)) {
      try {
        waypoints_ = loadWaypoints(params_.waypoints_file);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Cannot load waypoints: %s", e.what());
        throw;
      }
    } else {
      RCLCPP_WARN(get_logger(), "Starting with no waypoints: %s", params_.waypoints_file.c_str());
    }
    publisher_ = create_publisher<nav_waypoint_msgs::msg::Waypoints>(
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
    reserve_service_ = create_service<nav_waypoint_msgs::srv::ReserveRoute>(
      "~/reserve_route", [this](nav_waypoint_msgs::srv::ReserveRoute::Request::ConstSharedPtr req,
      nav_waypoint_msgs::srv::ReserveRoute::Response::SharedPtr response) {
        try {
          if (req->owner.empty() || (!req->route.header.frame_id.empty() &&
          req->route.header.frame_id != params_.waypoint_frame_id))
          {
            throw std::invalid_argument("Invalid route owner or frame");
          }
          std::unordered_set<std::string> known, references;
          for (const auto & w : waypoints_.waypoints) {known.insert(w.name);}
          for (const auto & name : req->route.route) {
            if (!known.count(name)) {
              throw std::invalid_argument("Unknown route waypoint: " + name);
            }
            references.insert(name);
          }
          route_references_[req->owner] = std::move(references);
          response->success = true;
          response->message = "Route references reserved";
        } catch (const std::exception & e) {response->message = e.what();}
      });
    save_ = create_service<std_srvs::srv::Trigger>(
      "~/save", [this](rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
      std::shared_ptr<rmw_request_id_t> header, std_srvs::srv::Trigger::Request::SharedPtr) {
        const auto snapshot = waypoints_;
        const auto frame = params_.waypoint_frame_id;
        const auto path = params_.waypoints_save_file.empty() ?
        params_.waypoints_file : params_.waypoints_save_file;
        auto reply = [service, header](bool success, const std::string & message) {
          std_srvs::srv::Trigger::Response response;
          response.success = success;
          response.message = message;
          try {service->send_response(*header, response);} catch (const std::exception &) {}
        };
        if (!saver_.submit(
          [snapshot, frame, path, reply]() {
            try {
              saveWaypoints(snapshot, path, frame);
              reply(true, "Saved waypoints");
            } catch (const std::exception & e) {reply(false, e.what());}
          })) {reply(false, "Save queue is full; retry later");}
      });
    publish_ = create_service<std_srvs::srv::Empty>(
      "~/publish", [this](std_srvs::srv::Empty::Request::ConstSharedPtr,
      std_srvs::srv::Empty::Response::SharedPtr) {publishWaypoints();});
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
    delete_service_ = create_service<nav_waypoint_msgs::srv::DeleteWaypoint>(
      "~/delete", [this](nav_waypoint_msgs::srv::DeleteWaypoint::Request::ConstSharedPtr request,
      nav_waypoint_msgs::srv::DeleteWaypoint::Response::SharedPtr response) {
        if (route_references_.empty()) {
          response->message = "Route references have not been received yet";
          return;
        }
        for (const auto & owner : route_references_) {
          if (owner.second.count(request->name)) {
            response->message = "Remove the waypoint from the route before deleting it";
            return;
          }
        }
        const auto found = std::find_if(
          waypoints_.waypoints.begin(), waypoints_.waypoints.end(),
          [&request](const nav_waypoint_msgs::msg::Waypoint & w) {return w.name == request->name;});
        if (found == waypoints_.waypoints.end()) {
          response->message = "Unknown waypoint: " + request->name;
          return;
        }
        waypoints_.waypoints.erase(found);
        publishWaypoints();
        response->success = true;
        response->message = "Deleted waypoint";
      });
    publishWaypoints();
  }

private:
  nav_waypoint_msgs::msg::Waypoints waypoints_;
  std::unordered_map<std::string, std::unordered_set<std::string>> route_references_;
  std::unique_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoints>::SharedPtr publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoint>::SharedPtr regist_, modify_;
  rclcpp::Service<nav_waypoint_msgs::srv::ReserveRoute>::SharedPtr reserve_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_;
  rclcpp::Service<nav_waypoint_msgs::srv::UpsertWaypoint>::SharedPtr upsert_service_;
  rclcpp::Service<nav_waypoint_msgs::srv::DeleteWaypoint>::SharedPtr delete_service_;
  AsyncSaver saver_;

  void publishWaypoints()
  {
    waypoints_.header.stamp = now();
    publisher_->publish(waypoints_);
  }

  void normalize(nav_waypoint_msgs::msg::Waypoint & w) const
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

  void upsert(nav_waypoint_msgs::msg::Waypoint w, bool require_existing)
  {
    nav_waypoint_msgs::validateWaypoint(w, params_.waypoint_frame_id);
    normalize(w);
    w.frame_id = params_.waypoint_frame_id;
    w.stamp = now();
    auto found = std::find_if(
      waypoints_.waypoints.begin(), waypoints_.waypoints.end(),
      [&w](const nav_waypoint_msgs::msg::Waypoint & current) {return current.name == w.name;});
    if (found == waypoints_.waypoints.end()) {
      if (require_existing) {
        throw std::invalid_argument("Unknown waypoint: " + w.name);
      }
      waypoints_.waypoints.push_back(w);
    } else {
      if (nav_waypoint_msgs::sameWaypoint(*found, w)) {return;}
      *found = w;
    }
    publishWaypoints();
  }

  nav_waypoint_msgs::msg::Waypoints loadWaypoints(const std::string & path)
  {
    const auto root = loadDataset(path, params_.waypoint_frame_id);
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

  static void saveWaypoints(
    const nav_waypoint_msgs::msg::Waypoints & snapshot,
    const std::string & path, const std::string & frame)
  {
    nav_waypoint_msgs::validateWaypoints(snapshot, frame);
    auto root = outputDataset(path, frame);
    YAML::Node entries(YAML::NodeType::Map);
    for (const auto & w : snapshot.waypoints) {
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
    atomicSave(path, root);
  }
};
}  // namespace nav_waypoint_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_server::NavWaypointServerNode)
