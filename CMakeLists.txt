# Establishes the minimum version of cmake needed to parse it in  
cmake_minimum_required(VERSION 2.8.3)
project(ping_driver)

# Finding other CMake / Catkin packages needed to build the current one 
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    rospy 
    std_msgs
    geometry_msgs
    message_generation
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/PingDriver.cfg)

# Dunno what these do but they're in the other cmake and look important 
include_directories(include ${catkin_INCLUDE_DIRS})

# Establishes the messages used by the package 
add_message_files (
    FILES
    pingMessage.msg
)

generate_messages()






