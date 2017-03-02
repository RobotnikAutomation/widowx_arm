cmake_minimum_required(VERSION 2.8.3)
project(widowx_arm_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy 
  arbotix_python
)

catkin_python_setup()
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/widowx_gripper.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
