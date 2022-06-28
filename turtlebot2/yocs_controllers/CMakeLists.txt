cmake_minimum_required(VERSION 3.0.2)
project(yocs_controllers)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp 
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
