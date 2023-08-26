#include <joint_trajectory_publisher_gui/shared_signal.hpp>


namespace joint_trajectory_publisher_gui
{
SharedSignal::SharedSignal(QObject * parent)
: QObject(parent)
{
}

SharedSignal::~SharedSignal() {}
}  // namespace joint_trajectory_publisher_gui
