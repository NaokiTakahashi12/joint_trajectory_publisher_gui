#include <joint_trajectory_publisher_gui/joint_trajectory_publisher_node.hpp>

#include <urdf_model/model.h>
#include <urdf_world/world.h>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <functional>
#include <utility>
#include <string>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


namespace joint_trajectory_publisher_gui
{
JointTrajectoryPublisherNode::JointTrajectoryPublisherNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("joint_trajectory_publisher", node_options),
  m_moveable_joint_names(),
  m_robot_description_subscriber(nullptr),
  m_joint_trajectory_publisher(nullptr),
  m_shared_signal(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

  m_robot_description_subscriber = this->create_subscription<std_msgs::msg::String>(
    "robot_description",
    rclcpp::QoS(1).transient_local(),
    std::bind(
      &JointTrajectoryPublisherNode::robotDescriptionSubscribeCallback,
      this,
      std::placeholders::_1
    )
  );
  m_joint_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory",
    rclcpp::QoS(5)
  );

  RCLCPP_INFO_STREAM(this->get_logger(), "Initialized successful " << this->get_name());
}

JointTrajectoryPublisherNode::~JointTrajectoryPublisherNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void JointTrajectoryPublisherNode::registerSharedSignal(SharedSignal::SharedPtr shared_signal)
{
  if (not shared_signal) {
    return;
  }
  if (m_shared_signal) {
    m_shared_signal.reset();
  }
  m_shared_signal = shared_signal;
  m_shared_signal->bindJointTrajectoryPublisher(
    [&](const std::vector<double> & dest_joint_anguler_positions) {
      if (dest_joint_anguler_positions.size() != m_moveable_joint_names.size()) {
        return;
      }
      if (not m_joint_trajectory_publisher) {
        return;
      }
      trajectory_msgs::msg::JointTrajectory::UniquePtr msg;
      msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();

      msg->header.stamp = this->get_clock()->now();
      msg->points.push_back(trajectory_msgs::msg::JointTrajectoryPoint());
      msg->joint_names = m_moveable_joint_names;
      for (const auto & dest_potision : dest_joint_anguler_positions) {
        msg->points.back().positions.push_back(dest_potision);
      }
      msg->points.back().time_from_start.sec = 1;
      msg->points.back().time_from_start.nanosec = 0;
      m_joint_trajectory_publisher->publish(std::move(msg));
    }
  );
}

std::vector<std::string> getMoveableJointNames(urdf::ModelInterfaceSharedPtr model_interface)
{
  std::vector<std::string> moveable_joint_names;
  for (const auto & [name, joint] : model_interface->joints_) {
    if (joint->type == urdf::Joint::FIXED || joint->type == urdf::Joint::UNKNOWN) {
      continue;
    }
    moveable_joint_names.push_back(name);
  }
  return moveable_joint_names;
}

void JointTrajectoryPublisherNode::robotDescriptionSubscribeCallback(
  std_msgs::msg::String::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Subscribed new robot description string");

  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty robot description detected");
    return;
  }
  urdf::ModelInterfaceSharedPtr model_interface = urdf::parseURDF(msg->data);
  m_moveable_joint_names.clear();
  m_moveable_joint_names = getMoveableJointNames(model_interface);

  if (m_shared_signal) {
    emit m_shared_signal->robotDescriptionUpdated(QString::fromStdString(model_interface->name_));
    {
      JointConfigurations joint_configs;

      for (const auto & joint_name : m_moveable_joint_names) {
        auto joint = model_interface->getJoint(joint_name);
        joint_configs.push_back(new JointConfiguration());
        joint_configs.last()->setName(QString::fromStdString(joint_name));

        if (not joint->limits) {
          joint_configs.last()->setLower(0);
          joint_configs.last()->setUpper(0);
        } else {
          joint_configs.last()->setLower(joint->limits->lower);
          joint_configs.last()->setUpper(joint->limits->upper);
        }
      }
      emit m_shared_signal->jointConfigurationChanged(joint_configs);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Joint trajectory publsiher configuration successful");
}
}  // namespace joint_trajectory_publisher_gui

RCLCPP_COMPONENTS_REGISTER_NODE(joint_trajectory_publisher_gui::JointTrajectoryPublisherNode)
