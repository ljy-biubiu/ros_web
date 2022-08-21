cmake_minimum_required(VERSION 3.5)
project(cti_mapsample)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color=always)
endif()

add_definitions(-DLINUX)


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

find_package(cti_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cti_vector_map_msgs REQUIRED)
find_package(cti_rblite_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED )
find_package(PCL REQUIRED)

find_package(yaml-cpp REQUIRED)

# SET( CMAKE_BUILD_TYPE "Debug")
# find_package(OpenCV 3.4.5 REQUIRED)

include_directories(
    include/
    include/FTP
    include/matToPng_base64
    # ${catkin_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mapsample.cpp
    src/FTP/FTPClient.cpp
    src/FTP/CurlHandle.cpp
)

# add_dependencies(${PROJECT_NAME}
#     ${catkin_EXPORTED_TARGETS}
# )

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${YAML_CPP_INCLUDE_DIRS}
    ${catkin_LIBRARIES}
    ${CURL_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    jsoncpp
  )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CURL_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    jsoncpp
)

ament_target_dependencies(${PROJECT_NAME}
  "ament_index_cpp"
  "cti_msgs"
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "tf2_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "diagnostic_msgs"
  "cti_vector_map_msgs"
  "cti_rblite_msgs"
  "cv_bridge"
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

#安装配置文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  include
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
