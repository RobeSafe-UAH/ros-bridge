cmake_minimum_required(VERSION 2.8.3)
project(t4ac_control_to_throttle)

# Find catkin macros and libraries
find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             rospy
             std_msgs
             carla_msgs
             roslaunch)

generate_messages(DEPENDENCIES std_msgs carla_msgs)

roslaunch_add_file_check(launch)

catkin_package()

install(PROGRAMS src/t4ac_control_to_throttle_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
