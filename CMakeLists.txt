cmake_minimum_required(VERSION 3.0.2)
project(wolf_controller_utils)

find_package(PkgConfig)
pkg_check_modules(EIGEN eigen3)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_eigen
    srdfdom
    urdf
)

include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        basefoot_estimator
        trajectory_utils
        pose_getter
        srdf_parser
    CATKIN_DEPENDS
        srdfdom
        urdf
    )

add_library(basefoot_estimator src/basefoot_estimator.cpp)
target_link_libraries(basefoot_estimator ${CATKIN_LIBRARIES})

add_library(trajectory_utils src/trajectory/trajectory.cpp)
target_link_libraries(trajectory_utils ${CATKIN_LIBRARIES})

add_library(pose_getter src/pose_getter.cpp)
target_link_libraries(pose_getter ${CATKIN_LIBRARIES})

add_library(srdf_parser src/srdf_parser.cpp)
target_link_libraries(srdf_parser ${CATKIN_LIBRARIES})

install(TARGETS basefoot_estimator trajectory_utils pose_getter srdf_parser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

