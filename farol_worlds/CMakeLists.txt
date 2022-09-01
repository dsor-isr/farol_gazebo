cmake_minimum_required(VERSION 3.1)
project(farol_worlds)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
catkin_package()
include_directories()

install(DIRECTORY launch Media worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
