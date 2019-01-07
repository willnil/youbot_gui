cmake_minimum_required(VERSION 2.8.3)
project(cv_pipeline)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  cv_camera
  OpenCV
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(FILES 
  FloatList.msg
  FloatArray.msg
)
  
generate_messages(DEPENDENCIES std_msgs)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

