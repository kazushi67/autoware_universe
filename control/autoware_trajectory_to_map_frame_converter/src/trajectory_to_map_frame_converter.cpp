#include "trajectory_to_map_frame_converter.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::control::trajectory_to_map_frame_converter
{

TrajectoryToMapFrameConverterNode::TrajectoryToMapFrameConverterNode(
  const rclcpp::NodeOptions & node_options)
: Node("trajectory_to_map_frame_converter", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", rclcpp::QoS(1),
    std::bind(&TrajectoryToMapFrameConverterNode::onTrajectory, this, std::placeholders::_1));

  pub_trajectory_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", rclcpp::QoS(1));
}

void TrajectoryToMapFrameConverterNode::onTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  // Ignore the frame_id embedded in the message.
  // Always apply the TF from base_link to map.

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to look up transform from 'base_link' to 'map': %s", ex.what());
    return;
  }

  ++msg_count_;

  if (msg->points.size() < 2) {
    return;
  }

  // Vehicle position in map frame — used as the pivot for position mirroring.
  const double vx = tf_stamped.transform.translation.x;
  const double vy = tf_stamped.transform.translation.y;

  // Transform each point from base_link frame to map frame,
  // then mirror position around the vehicle to correct the reversed path direction.
  auto transformed = *msg;

  transformed.header.frame_id = "map";
  for (auto & point : transformed.points) {
    geometry_msgs::msg::PoseStamped pose_in;
    geometry_msgs::msg::PoseStamped pose_out;
    pose_in.pose = point.pose;

    // Step 3: TF transform (base_link → map).
    tf2::doTransform(pose_in, pose_out, tf_stamped);

    // Step 4: mirror position 180 degrees around vehicle position.
    //   p_new = vehicle_pos - (p - vehicle_pos) = 2*vehicle_pos - p
    const double ddx = pose_out.pose.position.x - vx;
    const double ddy = pose_out.pose.position.y - vy;
    pose_out.pose.position.x = vx - ddx;
    pose_out.pose.position.y = vy - ddy;

    point.pose = pose_out.pose;
  }

  // Discard trajectories whose heading is opposite to the vehicle's heading in map frame.
  // Compare the yaw of the first converted trajectory point with the vehicle yaw.
  // If the difference exceeds 90 degrees, the trajectory is heading in the wrong direction.
  const double vehicle_yaw = tf2::getYaw(tf_stamped.transform.rotation);
  const double traj_yaw    = tf2::getYaw(transformed.points.front().pose.orientation);
  double yaw_diff = traj_yaw - vehicle_yaw;
  // Normalize to [-π, π]
  while (yaw_diff >  M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
  if (std::abs(yaw_diff) > M_PI / 2.0) {
    RCLCPP_ERROR(
      get_logger(),
      "Discarded: trajectory heading (%.1f deg) is opposite to vehicle heading (%.1f deg), "
      "yaw_diff=%.1f deg (msg_count=%lu).",
      traj_yaw * 180.0 / M_PI,
      vehicle_yaw * 180.0 / M_PI,
      yaw_diff * 180.0 / M_PI,
      msg_count_);
    return;
  }

  // Remove leading zero-velocity points so the PID longitudinal controller does not
  // interpret the trajectory's initial v=0 (current vehicle state) as a stop destination.
  // searchZeroVelocityIndex uses the same threshold: v < 1e-3 m/s.
  static constexpr float kZeroVelThreshold = 0.01;
  const bool has_nonzero_vel = std::any_of(
    transformed.points.begin(), transformed.points.end(),
    [](const auto & pt) { return pt.longitudinal_velocity_mps > kZeroVelThreshold; });
  if (has_nonzero_vel) {
    auto first_nonzero = std::find_if(
      transformed.points.begin(), transformed.points.end(),
      [](const auto & pt) { return pt.longitudinal_velocity_mps > kZeroVelThreshold; });
    transformed.points.erase(transformed.points.begin(), first_nonzero);
  }

  if (transformed.points.size() < 2) {
    return;
  }

  pub_trajectory_->publish(transformed);
}

}  // namespace autoware::control::trajectory_to_map_frame_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::control::trajectory_to_map_frame_converter::TrajectoryToMapFrameConverterNode)
