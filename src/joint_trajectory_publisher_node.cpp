#include <joint_trajectory_publisher_gui/joint_trajectory_publisher_node.hpp>

#include <urdf_model/model.h>
#include <urdf_world/world.h>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <functional>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


namespace joint_trajectory_publisher_gui
{
JointTrajectoryPublisherNode::JointTrajectoryPublisherNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("joint_trajectory_publisher", node_options),
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
  m_shared_signal = shared_signal;
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
  RCLCPP_INFO_STREAM(this->get_logger(), model_interface->joints_.size());
  RCLCPP_INFO(this->get_logger(), "Joint trajectory publsiher configuration successful");
}
}  // namespace joint_trajectory_publisher_gui

RCLCPP_COMPONENTS_REGISTER_NODE(joint_trajectory_publisher_gui::JointTrajectoryPublisherNode)
