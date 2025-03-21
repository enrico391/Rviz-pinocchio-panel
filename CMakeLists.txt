cmake_minimum_required(VERSION 3.8)
project(rviz_panel_pinocchio_tiago)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/rviz_panel_pinocchio_tiago/pinocchio_panel_manage.hpp
)

add_library(pinocchio_panel_manage src/pinocchio_panel_manage.cpp src/pinocchio_manage.cpp ${MOC_FILES})

target_include_directories(pinocchio_panel_manage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pinocchio_panel_manage
  pluginlib
  rviz_common
  ament_index_cpp
  Eigen3
  eigen3_cmake_module
  rclcpp
  sensor_msgs
)

target_link_libraries(pinocchio_panel_manage pinocchio::pinocchio )

install(TARGETS pinocchio_panel_manage
        EXPORT export_rviz_panel_pinocchio_tiago
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include 
)
install(
  DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/pinocchio_img.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_panel_pinocchio_tiago)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
