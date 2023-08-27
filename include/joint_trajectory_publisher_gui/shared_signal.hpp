#pragma once

#include <QObject>

#include <memory>
#include <functional>
#include <utility>
#include <vector>

#include <QString>
#include <QVector>

#include "joint_configuration.hpp"


namespace joint_trajectory_publisher_gui
{
//! @todo remove or generate this class
class SharedSignal : public QObject
{
  Q_OBJECT

public:
  using SharedPtr = std::shared_ptr<SharedSignal>;
  using JointAngulerPositions = std::vector<double>;
  using PublishJointTrajectoryPositionFunc = std::function<void (const JointAngulerPositions &)>;

  SharedSignal(QObject * parent = nullptr);
  ~SharedSignal();

  void bindJointTrajectoryPublisher(PublishJointTrajectoryPositionFunc);

public slots:
  void publishJointAngulerPositions(const JointAngulerPositions &);

signals:
  void robotDescriptionUpdated(const QString &);
  void jointConfigurationChanged(const JointConfigurations &);

private:
  PublishJointTrajectoryPositionFunc m_publish_joint_trajectory;
};
}  // namespace joint_trajectory_publisher_gui
