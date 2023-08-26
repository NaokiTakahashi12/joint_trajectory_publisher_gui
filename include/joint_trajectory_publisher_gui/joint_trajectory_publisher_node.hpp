#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "shared_signal.hpp"


namespace joint_trajectory_publisher_gui
{
class JointTrajectoryPublisherNode : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<JointTrajectoryPublisherNode>;

  JointTrajectoryPublisherNode(const rclcpp::NodeOptions &);
  ~JointTrajectoryPublisherNode();

  void registerSharedSignal(SharedSignal::SharedPtr);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_description_subscriber;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_publisher;

  SharedSignal::SharedPtr m_shared_signal;

  void robotDescriptionSubscribeCallback(std_msgs::msg::String::ConstSharedPtr);
};
}  // namespace joint_trajectory_publisher_gui
