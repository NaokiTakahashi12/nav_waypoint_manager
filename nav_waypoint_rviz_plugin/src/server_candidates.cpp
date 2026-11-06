// Copyright (c) 2026 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "server_candidates.hpp"

#include <QComboBox>
#include <QSignalBlocker>
#include <QStringList>
#include <algorithm>
#include <rclcpp/node.hpp>

namespace nav_waypoint_rviz_plugin
{
void refreshServerCandidates(
  QComboBox & selector, const rclcpp::Node & node,
  const std::string & topic_suffix, const std::string & message_type)
{
  QStringList candidates;
  const auto suffix = QString::fromStdString(topic_suffix);
  for (const auto & topic : node.get_topic_names_and_types()) {
    const auto name = QString::fromStdString(topic.first);
    if (name.size() <= suffix.size() || !name.endsWith(suffix)) {continue;}
    const auto publishers = node.get_publishers_info_by_topic(topic.first);
    if (std::any_of(
        publishers.begin(), publishers.end(), [&message_type](const auto & publisher) {
          return publisher.topic_type() == message_type;
        }))
    {
      candidates.push_back(name.left(name.size() - suffix.size()));
    }
  }
  candidates.sort();
  const auto current = selector.currentText();
  QSignalBlocker blocker(selector);
  selector.clear();
  selector.addItems(candidates);
  selector.setCurrentIndex(selector.findText(current));
  selector.setEditText(current);
}
}  // namespace nav_waypoint_rviz_plugin
