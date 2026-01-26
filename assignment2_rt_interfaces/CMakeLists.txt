cmake_minimum_required(VERSION 3.8)
project(assignment2_rt_interfaces)

# Colcon (per compatibilità) passa -DCATKIN_INSTALL_INTO_PREFIX_ROOT=...
# Non ci serve, ma la "consumiamo" per evitare warning "unused by the project".
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(_catkin_install_into_prefix_root "${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
endif()
if(DEFINED CATKIN_SYMLINK_INSTALL)
  set(_catkin_symlink_install "${CATKIN_SYMLINK_INSTALL}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetVelAvg.srv"
)

# Install esplicito dei file interface (ok anche con merge-install)
install(DIRECTORY msg srv DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
