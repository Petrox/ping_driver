cmake_minimum_required(VERSION 2.8.3)
project(ping_driver)

find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
    FILES
    depth.msg
    std_msgs
)
