cmake_minimum_required(VERSION 3.0.2)
project(led_circuit_ros)

find_package(catkin REQUIRED
  catkin_virtualenv
)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3.8
)

catkin_package()

catkin_python_setup()

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS scripts/led_circuit_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
