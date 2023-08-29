// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_util/geometry_utils.hpp"

namespace nav2_util
{
namespace geometry_utils
{

double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>
find_closest_path_segment(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (path.poses.empty()) {
    throw std::invalid_argument("No poses on path to project onto");
  }

  if (path.poses.size() == 1u) {
    return std::make_pair(*path.poses.begin(), *path.poses.begin());
  }

  std::vector<double> distances(path.poses.size());
  std::transform(
    path.poses.begin(),
    path.poses.end(),
    distances.begin(),
    [&pose](const auto & path_pose) {
      return squared_euclidean_distance(path_pose, pose);
    }
  );
  // We want to find the first local minima,
  // so we don't loop around and get something from later
  // if we start out by increasing distance away,
  // then we will accept the beginning
  auto distance_it = distances.begin();
  for (; distance_it != std::prev(distances.end()); ++distance_it) {
    double delta = *std::next(distance_it) - *distance_it;
    if (delta > 0.0) {
      break;
    }
  }
  auto first_minimum_it = distance_it;

  auto next_closest_distance_it = distances.end();

  if (std::next(first_minimum_it) == distances.end()) {
    next_closest_distance_it = std::prev(first_minimum_it);
  } else if (first_minimum_it == distances.begin()) {
    next_closest_distance_it = std::next(first_minimum_it);
  } else if (*std::prev(first_minimum_it) < *std::next(first_minimum_it)) {
    next_closest_distance_it = std::prev(first_minimum_it);
  } else {
    next_closest_distance_it = std::next(first_minimum_it);
  }

  size_t closest_pose_index = std::distance(distances.begin(), first_minimum_it);
  size_t next_closest_pose_index = std::distance(distances.begin(), next_closest_distance_it);

  return std::make_pair(path.poses.at(closest_pose_index), path.poses.at(next_closest_pose_index));
}

/**
 * Assuming the robot is at the origin of the frame the path is in,
 * find the closest pose on the path to the robot, interpolated in both position and orientation.
*/
geometry_msgs::msg::PoseStamped project_robot_onto_path(const nav_msgs::msg::Path & path)
{
  // http://paulbourke.net/geometry/pointlineplane/

  // Even if the target pose is "off the end" of this line segment, the projected point
  // exists as if the segment was extended out.
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  const auto & [pose1, pose2] = find_closest_path_segment(path, robot_pose);
  const double x1 = pose1.pose.position.x;
  const double y1 = pose1.pose.position.y;
  const double x2 = pose2.pose.position.x;
  const double y2 = pose2.pose.position.y;
  const double x3 = robot_pose.pose.position.x;
  const double y3 = robot_pose.pose.position.y;

  const double segment_length_squared = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  double u;
  if (segment_length_squared == 0.0) {
    u = 1.0;  // We can just pick p2 as the projection point since they are the same
  } else {
    u = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) / segment_length_squared;
  }

  const double x = x1 + u * (x2 - x1);
  const double y = y1 + u * (y2 - y1);

  tf2::Quaternion q1;
  tf2::Quaternion q2;
  tf2::fromMsg(pose1.pose.orientation, q1);
  tf2::fromMsg(pose2.pose.orientation, q2);

  // u is the ratio of p1 to p2, which works perfectly for slerp
  tf2::Quaternion q = tf2::slerp(q1, q2, u);

  geometry_msgs::msg::PoseStamped projected_pose;
  projected_pose.header.frame_id = pose1.header.frame_id;
  projected_pose.pose.position.x = x;
  projected_pose.pose.position.y = y;
  projected_pose.pose.position.z = pose1.pose.position.z;
  projected_pose.pose.orientation = tf2::toMsg(q);

  return projected_pose;
}

nav2_msgs::msg::CrossTrackError calculate_cross_track_error(
  geometry_msgs::msg::PoseStamped projected_pose)
{
  nav2_msgs::msg::CrossTrackError error;
  error.position_error = std::hypot(projected_pose.pose.position.x, projected_pose.pose.position.y);
  tf2::Quaternion error_orientation;
  tf2::fromMsg(projected_pose.pose.orientation, error_orientation);
  error.heading_error = tf2::getYaw(error_orientation);
  return error;
}

}  // namespace geometry_utils
}  // namespace nav2_util
