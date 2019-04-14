cmake_minimum_required(VERSION 2.8.3)
project(ping_driver)

# Finding other CMake / Catkin packages needed to build the current one 
find_package(catkin REQUIRED 
    COMPONENTS
    rospy 
    std_msgs
    message_generation
    dynamic_reconfigure
)

# Links the messages used by the node 
add_message_files (FILES pingMessage.msg)

# Sets up project imports using information from setup.py
catkin_python_setup()

generate_messages()

# Needs called before catkin_package 
generate_dynamic_reconfigure_options(cfg/PingDriverConfig.cfg)

# Must be run before catkin_install_python as it defines CATKIN_PACKAGE_BIN_DESTINATION
catkin_package()

catkin_install_python(programs scripts/pingDriver.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_dependencies(ping_driver ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)













