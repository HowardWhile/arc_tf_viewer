cmake_minimum_required(VERSION 3.8)
project(arc_tf_viewer)

# ------------------------------------------
# Use C++ 17
# ------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------
# Set compilation type and dependencies
# ------------------------------------------
# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Qt
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# ------------------------------------------
# Add executable
# ------------------------------------------
add_executable(ui 
  src/main.cpp
  src/mainwindow.cpp
  src/static_tf_panel.cpp
  src/static_tf_panel.h
  src/tf_preview.cpp
  src/tf_preview.h
  src/mainwindow.h
  src/mainwindow.ui
  src/ArcTFTree.hpp
  icons/icons.qrc
)

# ------------------------------------------
# Link dependencies
# ------------------------------------------
# ROS
target_link_libraries(ui 
  ${rclcpp_LIBRARIES} 
  yaml-cpp
)
# Qt
target_link_libraries(ui 
  Qt5::Core 
  Qt5::Gui
  Qt5::Widgets
)

# ------------------------------------------
# Include header paths of other packages
# ------------------------------------------
ament_target_dependencies(ui 
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ------------------------------------------
# Set up automatic MOC, UIC, and RCC (related to Qt)
# ------------------------------------------
set_target_properties(ui PROPERTIES AUTOMOC ON)
set_target_properties(ui PROPERTIES AUTOUIC ON)
set_target_properties(ui PROPERTIES AUTORCC ON)
# ------------------------------------------

# Install executable
install(TARGETS ui
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  enable_testing()
  find_package(Qt5 REQUIRED COMPONENTS Test)
  find_package(tf2_msgs REQUIRED)
  find_package(urdf REQUIRED)
  add_executable(static_tf_panel_test tests/static_tf_panel_test.cpp src/static_tf_panel.cpp src/static_tf_panel.h src/tf_preview.cpp src/tf_preview.h)
  target_include_directories(static_tf_panel_test PRIVATE src)
  target_link_libraries(static_tf_panel_test Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Test yaml-cpp)
  ament_target_dependencies(static_tf_panel_test rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs tf2_msgs urdf)
  set_target_properties(static_tf_panel_test PROPERTIES AUTOMOC ON)
  add_test(NAME static_tf_panel_test COMMAND static_tf_panel_test)
  set_tests_properties(static_tf_panel_test PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=137;XDG_CONFIG_HOME=/tmp/arc_tf_viewer_test_config")
endif()

ament_package()
