#include <joint_trajectory_publisher_gui/joint_configuration.hpp>

namespace joint_trajectory_publisher_gui
{
JointConfiguration::JointConfiguration(QObject * parent)
: QObject(parent) {}

JointConfiguration::~JointConfiguration() {}

QString JointConfiguration::getName() const
{
  return m_name;
}

double JointConfiguration::getPosition() const
{
  return m_position;
}

double JointConfiguration::getLower() const
{
  return m_lower;
}

double JointConfiguration::getUpper() const
{
  return m_upper;
}

void JointConfiguration::setName(const QString & name)
{
  m_name = name;
}

void JointConfiguration::setPosition(const double position)
{
  m_position = position;
}

void JointConfiguration::setLower(const double lower)
{
  m_lower = lower;
}

void JointConfiguration::setUpper(const double upper)
{
  m_upper = upper;
}
}  // namespace joint_trajectory_publisher_gui
