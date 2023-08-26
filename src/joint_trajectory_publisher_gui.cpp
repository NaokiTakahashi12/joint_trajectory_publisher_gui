#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <rclcpp/rclcpp.hpp>

#include <joint_trajectory_publisher_gui/shared_signal.hpp>
#include <joint_trajectory_publisher_gui/joint_trajectory_publisher_node.hpp>
#include <joint_trajectory_publisher_gui/node_spinner.hpp>


auto main(int argc, char ** argv) -> int
{
  const QUrl main_qml_url(
    u"qrc:/joint_trajectory_publisher_gui/qml/joint_trajectory_publisher_gui.qml"_qs);

  QGuiApplication gui_app(argc, argv);
  QQmlApplicationEngine qml_app_engine;

  joint_trajectory_publisher_gui::NodeSpinner::SharedPtr node_spinner;
  joint_trajectory_publisher_gui::SharedSignal::SharedPtr shared_node_signal;
  std::shared_ptr<joint_trajectory_publisher_gui::JointTrajectoryPublisherNode> rcl_node;

  shared_node_signal = std::make_shared<joint_trajectory_publisher_gui::SharedSignal>(
    &gui_app
  );
  node_spinner = std::make_shared<joint_trajectory_publisher_gui::NodeSpinner>(
    argc,
    argv,
    &gui_app
  );
  rcl_node = std::make_shared<joint_trajectory_publisher_gui::JointTrajectoryPublisherNode>(
    node_spinner->getNodeOptions()
  );
  rcl_node->registerSharedSignal(shared_node_signal);
  node_spinner->addNode(rcl_node);

  qml_app_engine.rootContext()->setContextProperty("nodeSharedSignal", shared_node_signal.get());
  qml_app_engine.rootContext()->setContextProperty("nodeSpinner", node_spinner.get());

  QObject::connect(
    &qml_app_engine,
    &QQmlApplicationEngine::objectCreated,
    &gui_app,
    [main_qml_url](QObject * obj, const QUrl & obj_url) {
      if (!obj && main_qml_url == obj_url) {
        QCoreApplication::exit(-1);
      }
    },
    Qt::QueuedConnection
  );

  qml_app_engine.load(main_qml_url);

  return gui_app.exec();
}
