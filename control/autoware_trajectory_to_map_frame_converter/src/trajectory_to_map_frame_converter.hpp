#ifndef TRAJECTORY_TO_MAP_FRAME_CONVERTER_HPP_
#define TRAJECTORY_TO_MAP_FRAME_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_planning_msgs/msg/trajectory.hpp>

namespace autoware::control::trajectory_to_map_frame_converter
{

class TrajectoryToMapFrameConverterNode : public rclcpp::Node
{
public:
  explicit TrajectoryToMapFrameConverterNode(const rclcpp::NodeOptions & node_options);

private:
  // subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;

  // publisher
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // received message counter
  uint64_t msg_count_{0};

  void onTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
};

}  // namespace autoware::control::trajectory_to_map_frame_converter

#endif  // TRAJECTORY_TO_MAP_FRAME_CONVERTER_HPP_
