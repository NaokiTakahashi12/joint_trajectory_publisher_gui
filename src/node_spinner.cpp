#include <joint_trajectory_publisher_gui/node_spinner.hpp>

#include <rclcpp/rclcpp.hpp>


namespace joint_trajectory_publisher_gui
{
NodeSpinner::NodeSpinner(int argc, char ** argv, QObject * parent)
: QObject(parent),
  m_this_name("node_spinner"),
  m_node_options(),
  m_logger(rclcpp::get_logger(m_this_name)),
  m_executor(nullptr)
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  m_executor = std::make_unique<RCLExecutor>();

  m_node_options.arguments(args);
}

NodeSpinner::~NodeSpinner() {}

const rclcpp::NodeOptions & NodeSpinner::getNodeOptions() const
{
  return m_node_options;
}

void NodeSpinner::addNode(rclcpp::Node::SharedPtr node)
{
  if (not m_executor) {
    RCLCPP_WARN(m_logger, "Executor is nullptr");
    return;
  }
  m_executor->add_node(node);
}

void NodeSpinner::spinSome()
{
  if (not m_executor) {
    RCLCPP_WARN(m_logger, "Executor is nullptr");
    return;
  }
  m_executor->spin_some();
}
}  // namespace joint_trajectory_publisher_gui
