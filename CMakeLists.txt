cmake_minimum_required(VERSION 3.0.2)
project(wolf_controller_utils)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

find_package(PkgConfig)
pkg_check_modules(EIGEN eigen3)

include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES basefoot_estimator
    )

add_library(basefoot_estimator src/basefoot_estimator.cpp)
target_link_libraries(basefoot_estimator ${CATKIN_LIBRARIES})

install(TARGETS basefoot_estimator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

