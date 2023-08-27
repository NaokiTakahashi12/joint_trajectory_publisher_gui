#pragma once

#include <string>
#include <vector>

#include <QObject>
#include <QList>


namespace joint_trajectory_publisher_gui
{
class JointConfiguration : public QObject
{
  Q_OBJECT

  Q_PROPERTY(QString name READ getName WRITE setName NOTIFY nameChanged)
  Q_PROPERTY(double position READ getPosition WRITE setPosition NOTIFY positionChanged)
  Q_PROPERTY(double lower READ getLower WRITE setLower NOTIFY lowerChanged)
  Q_PROPERTY(double upper READ getUpper WRITE setUpper NOTIFY upperChanged)

public:
  JointConfiguration(QObject * parent = nullptr);
  ~JointConfiguration();

  QString getName() const;
  double getPosition() const;
  double getLower() const;
  double getUpper() const;

  void setName(const QString &);
  void setPosition(const double);
  void setLower(const double);
  void setUpper(const double);

signals:
  void nameChanged();
  void positionChanged();
  void lowerChanged();
  void upperChanged();

private:
  QString m_name;
  double m_position;
  double m_lower;
  double m_upper;
};

using JointConfigurations = QList<JointConfiguration *>;
}  // namespace joint_trajectory_publisher_gui
