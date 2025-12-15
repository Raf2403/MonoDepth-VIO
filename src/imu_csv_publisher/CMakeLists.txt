cmake_minimum_required(VERSION 3.0.2)
project(imu_csv_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Устанавливаем скрипты, чтобы ROS их видел
install(PROGRAMS
  scripts/imu_data_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

