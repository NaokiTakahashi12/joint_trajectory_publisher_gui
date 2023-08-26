#pragma once

#include <QObject>

#include <memory>
#include <functional>
#include <utility>


namespace joint_trajectory_publisher_gui
{
class SharedSignal : public QObject
{
  Q_OBJECT

public:
  using SharedPtr = std::shared_ptr<SharedSignal>;

  SharedSignal(QObject * parent = nullptr);
  ~SharedSignal();
};
}  // namespace joint_trajectory_publisher_gui
