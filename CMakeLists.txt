cmake_minimum_required(VERSION 3.0.2)
project(progetto_esame)


find_package(catkin REQUIRED COMPONENTS
cv_bridge
image_transport
roscpp
sensor_msgs
std_msgs
roslib
kdl_parser
tf
nav_msgs
geometry_msgs
message_generation
)
find_library(ZBAR_LIBRARIES NAMES zbar)


add_message_files(
  FILES
  planner_msg.msg
  odom_msg.msg
  bag_msg.msg
  bag_msg1.msg
  bag_msg2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
)


catkin_package(
#  LIBRARIES progetto_esame
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
)

add_executable( opencv_node src/opencv_node.cpp)
target_link_libraries( opencv_node   ${catkin_LIBRARIES} opencv_highgui zbar)

add_executable( ctrl_node src/ctrl_node.cpp)
target_link_libraries ( ctrl_node ${catkin_LIBRARIES}  )

add_executable( planner_node src/planner_node.cpp)
target_link_libraries ( planner_node ${catkin_LIBRARIES}  )

add_executable( odom_node src/odom_node.cpp)
target_link_libraries ( odom_node ${catkin_LIBRARIES}  )

