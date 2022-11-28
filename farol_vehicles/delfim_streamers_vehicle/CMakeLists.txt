cmake_minimum_required(VERSION 3.1)
project(delfim_streamers_vehicle)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY meshes robots urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


install(DIRECTORY launch meshes robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
