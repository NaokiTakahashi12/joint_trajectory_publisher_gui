#pragma once

#include <QObject>

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/executors.hpp>


namespace joint_trajectory_publisher_gui
{
class NodeSpinner : public QObject
{
  Q_OBJECT

public:
  using SharedPtr = std::shared_ptr<NodeSpinner>;

  NodeSpinner(int argc, char ** argv, QObject * parent = nullptr);
  ~NodeSpinner();

  const rclcpp::NodeOptions & getNodeOptions() const;
  void addNode(rclcpp::Node::SharedPtr);

public slots:
  void spinSome();

private:
  using RCLExecutor = rclcpp::executors::SingleThreadedExecutor;

  const std::string m_this_name;

  rclcpp::NodeOptions m_node_options;
  rclcpp::Logger m_logger;
  std::unique_ptr<RCLExecutor> m_executor;
};
}  // namespace joint_trajectory_publisher_gui
