#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

#include <QObject>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "joint_configuration.hpp"


namespace joint_trajectory_publisher_gui
{
class JointTrajectoryPublisherNode : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  using SharedPtr = std::shared_ptr<JointTrajectoryPublisherNode>;

  JointTrajectoryPublisherNode(const rclcpp::NodeOptions &, QObject * parent = nullptr);
  ~JointTrajectoryPublisherNode();

public slots:
  void publishJointAngulerPositions(const std::vector<double> &);

signals:
  void robotDescriptionUpdated(const QString &);
  void jointConfigurationChanged(const JointConfigurations &);

private:
  std::vector<std::string> m_moveable_joint_names;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_description_subscriber;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_publisher;

  void robotDescriptionSubscribeCallback(std_msgs::msg::String::ConstSharedPtr);
};
}  // namespace joint_trajectory_publisher_gui
