cmake_minimum_required(VERSION 3.0.2)
project(my_robot_finder)

## Encontrar los paquetes requeridos de ROS
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  nav_msgs
  move_base_msgs
)

## Declarar el paquete catkin
catkin_package()

## Incluir directorios
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Instalar scripts de Python
catkin_install_python(PROGRAMS
  src/main.py
  src/iker.py
  src/nico.py
  src/klau.py
  src/otto.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
