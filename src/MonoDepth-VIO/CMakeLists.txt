cmake_minimum_required(VERSION 3.0.2)
project(MonoDepth-VIO)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
# Разрешаем устанавливать launch файлы и скрипты
install(DIRECTORY launch config scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
