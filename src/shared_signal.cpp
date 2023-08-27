#include <joint_trajectory_publisher_gui/shared_signal.hpp>

#include <stdexcept>


namespace joint_trajectory_publisher_gui
{
SharedSignal::SharedSignal(QObject * parent)
: QObject(parent)
{
}

SharedSignal::~SharedSignal() {}

void SharedSignal::bindJointTrajectoryPublisher(PublishJointTrajectoryPositionFunc func)
{
  if (not func) {
    throw std::runtime_error("Not callable function");
  }
  m_publish_joint_trajectory = func;
}

void SharedSignal::publishJointAngulerPositions(const std::vector<double> & positions)
{
  if (not m_publish_joint_trajectory) {
    return;
  }
  m_publish_joint_trajectory(positions);
}
}  // namespace joint_trajectory_publisher_gui
