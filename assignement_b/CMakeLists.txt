cmake_minimum_required(VERSION 2.8.3)
project(assignement_b)


# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(${PROJECT_NAME}_node src/reactive_turtlebot.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})