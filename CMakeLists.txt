cmake_minimum_required(VERSION 3.15)
project(joint_trajectory_publisher_gui VERSION 0.0.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(gui_executable gui)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Qt6 6.2 REQUIRED
  COMPONENTS
    Core
    Gui
    Quick
    Qml
)
set(qt6_libraries
  Qt6::Core
  Qt6::Gui
  Qt6::Quick
  Qt6::Qml
)

ament_auto_find_build_dependencies()

qt_add_library(shared_signal
  include/joint_trajectory_publisher_gui/shared_signal.hpp
  src/shared_signal.cpp
)
target_include_directories(shared_signal
  PUBLIC
    include
)
target_link_libraries(shared_signal
  ${qt6_libraries}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/joint_trajectory_publisher_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${qt6_libraries}
  shared_signal
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "joint_trajectory_publisher_gui::JointTrajectoryPublisherNode"
  EXECUTABLE joint_trajectory_publisher_node
)

qt_add_executable(${gui_executable}
  include/joint_trajectory_publisher_gui/node_spinner.hpp
  src/node_spinner.cpp
  src/joint_trajectory_publisher_gui.cpp
)
qt_add_qml_module(${gui_executable}
  URI ${PROJECT_NAME}
  VERSION ${PROJECT_VERSION}
  QML_FILES
    qml/joint_trajectory_publisher_gui.qml
    qml/Shortcuts.qml
    qml/JointTrajectory.qml
)
target_link_libraries(${gui_executable}
  PRIVATE
    ${qt6_libraries}
    ${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${gui_executable}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION "lib/${PROJECT_NAME}"
)
install(
  DIRECTORY include
  DESTINATION include
)
install(
  DIRECTORY
    qml
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
