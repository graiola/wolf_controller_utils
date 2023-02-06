cmake_minimum_required(VERSION 3.0.2)
project(wolf_controller_utils)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

