cmake_minimum_required(VERSION 2.8.3)
project(dwa_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2_ros
  usv_navigation_msgs
  dynamic_reconfigure
  message_filters
  quaternion_operation
  tf2_geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/DwaPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dwa_planner
  CATKIN_DEPENDS geometry_msgs roscpp tf2_ros usv_navigation_msgs tf2_geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(dwa_planner_node src/dwa_planner_node.cpp src/dwa_planner.cpp)
target_link_libraries(dwa_planner_node ${catkin_LIBRARIES})
add_dependencies(dwa_planner_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
set(NODE_FILES
  dwa_planner_node
  )
foreach(nodes ${NODE_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(nodes)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
