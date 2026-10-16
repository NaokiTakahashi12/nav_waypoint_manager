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

#ifndef DATASET_IO_HPP_
#define DATASET_IO_HPP_

#include <unistd.h>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace nav_waypoint_server
{
inline bool fileExists(const std::string & path)
{
  struct stat info;
  if (::stat(path.c_str(), &info) == 0) {
    return true;
  }
  if (errno == ENOENT) {
    return false;
  }
  throw std::runtime_error("Cannot inspect dataset: " + path);
}

inline YAML::Node loadDataset(const std::string & path, const std::string & frame)
{
  auto root = YAML::LoadFile(path);
  const auto manager = root["waypoint_manager"];
  if (!manager || !manager.IsMap()) {
    throw std::invalid_argument("Missing waypoint_manager mapping: " + path);
  }
  if (manager["frame_id"] && manager["frame_id"].as<std::string>() != frame) {
    throw std::invalid_argument("Dataset frame mismatch: " + path);
  }
  return root;
}

// Keep other dataset sections when a route and waypoints share an output file.
inline YAML::Node outputDataset(const std::string & path, const std::string & frame)
{
  YAML::Node root = fileExists(path) ? loadDataset(path, frame) : YAML::Node(YAML::NodeType::Map);
  root["waypoint_manager"]["frame_id"] = frame;
  return root;
}

inline void atomicSave(const std::string & path, const YAML::Node & root)
{
  if (path.empty()) {
    throw std::invalid_argument("Save path must not be empty");
  }
  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    throw std::runtime_error(emitter.GetLastError());
  }
  const std::string data = std::string(emitter.c_str()) + "\n";
  const std::string pattern = path + ".tmp.XXXXXX";
  std::vector<char> temporary(pattern.begin(), pattern.end());
  temporary.push_back('\0');
  int fd = ::mkstemp(temporary.data());
  if (fd < 0) {
    throw std::runtime_error("Cannot create save file: " + std::string(std::strerror(errno)));
  }
  try {
    std::size_t written = 0;
    while (written < data.size()) {
      const auto count = ::write(fd, data.data() + written, data.size() - written);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        throw std::runtime_error("Failed to write dataset");
      }
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(fd) != 0) {
      throw std::runtime_error("Failed to flush dataset");
    }
    const int close_result = ::close(fd);
    fd = -1;
    if (close_result != 0 || ::rename(temporary.data(), path.c_str()) != 0) {
      throw std::runtime_error("Failed to replace dataset");
    }
  } catch (...) {
    if (fd >= 0) {
      ::close(fd);
    }
    ::unlink(temporary.data());
    throw;
  }
}
}  // namespace nav_waypoint_server
#endif  // DATASET_IO_HPP_
