cmake_minimum_required(VERSION 3.5)
project(cti_mapsample)

add_compile_options(-std=c++14)
add_definitions(-DLINUX)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cti_msgs
    geometry_msgs
)

find_package(CURL REQUIRED)

include_directories(
    include/
    include/FTP
    ${catkin_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
)

catkin_package()

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mapsample.cpp
    src/FTP/FTPClient.cpp
    src/FTP/CurlHandle.cpp
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CURL_LIBRARIES}
    jsoncpp
)

#install
install(PROGRAMS scripts/state_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
