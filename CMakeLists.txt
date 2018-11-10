cmake_minimum_required(VERSION 2.8.3)
project(experimentos)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros
)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-std=c++0x -g -O3)

add_executable(uno src/uno.cpp)
add_executable(cero src/cero.cpp)
add_executable(dos src/dos.cpp)
add_executable(tres src/tres.cpp)
add_executable(ida_vuelta src/IV.cpp)
add_executable(giro_wp src/uno_wp.cpp)

target_link_libraries(uno ${catkin_LIBRARIES})
target_link_libraries(cero ${catkin_LIBRARIES})
target_link_libraries(dos ${catkin_LIBRARIES})
target_link_libraries(tres ${catkin_LIBRARIES})
target_link_libraries(ida_vuelta ${catkin_LIBRARIES})
target_link_libraries(giro_wp ${catkin_LIBRARIES})
