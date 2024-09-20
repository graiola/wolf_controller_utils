cmake_minimum_required(VERSION 3.5)
project(wolf_controller_utils)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)
if(catkin_FOUND)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES basefoot_estimator trajectory_utils pose_getter srdf_parser
        CATKIN_DEPENDS srdfdom urdf
        CFG_EXTRAS ros-extras.cmake
    )
    message(STATUS "Add ROS definition")
    add_compile_definitions(ROS)
elseif(ament_cmake_FOUND)
    ament_export_dependencies(srdfdom urdf)
    ament_export_include_directories(include)
    ament_export_libraries(basefoot_estimator trajectory_utils pose_getter srdf_parser)
    ament_package(CONFIG_EXTRAS cmake/ros2-extras.cmake)
    message(STATUS "Add ROS2 definition")
    add_compile_definitions(ROS2)
endif()

find_package(PkgConfig)
pkg_check_modules(EIGEN eigen3)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
)

# Define libraries and link the helper target to propagate the definitions
add_library(basefoot_estimator SHARED src/basefoot_estimator.cpp)

add_library(trajectory_utils SHARED src/trajectory/trajectory.cpp)

add_library(pose_getter SHARED src/pose_getter.cpp)
target_link_libraries(pose_getter ${geometry_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_eigen_LIBRARIES})

add_library(srdf_parser SHARED src/srdf_parser.cpp)
target_link_libraries(srdf_parser ${srdfdom_LIBRARIES} ${urdf_LIBRARIES})

# Installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(
  TARGETS basefoot_estimator trajectory_utils pose_getter srdf_parser
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
