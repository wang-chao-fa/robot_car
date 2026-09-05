cmake_minimum_required(VERSION 3.5)
project(openslam_gmapping)

# 使用 ament 构建系统
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include(GenerateExportHeader)
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成导出头文件
set(EXPORT_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/export_include")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}")

# 包含目录
include_directories(
  include
  ${EXPORT_HEADER_DIR}
)

###########
## Build ##
###########

# 添加编译定义确保符号可见性
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

# utils/
add_library(utils STATIC
  utils/stat.cpp
  utils/movement.cpp)
target_include_directories(utils PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# sensor/sensor_base/
add_library(sensor_base STATIC
  sensor/sensor_base/sensor.cpp
  sensor/sensor_base/sensorreading.cpp)
target_include_directories(sensor_base PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# sensor/sensor_odometry/
add_library(sensor_odometry STATIC
  sensor/sensor_odometry/odometrysensor.cpp
  sensor/sensor_odometry/odometryreading.cpp)
target_link_libraries(sensor_odometry sensor_base)
target_include_directories(sensor_odometry PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# sensor/sensor_range/
add_library(sensor_range STATIC
  sensor/sensor_range/rangesensor.cpp
  sensor/sensor_range/rangereading.cpp)
target_link_libraries(sensor_range sensor_base)
target_include_directories(sensor_range PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# log/
add_library(log STATIC
  log/configuration.cpp
  log/carmenconfiguration.cpp
  log/sensorlog.cpp
  log/sensorstream.cpp)
target_include_directories(log PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# configfile/
add_library(configfile STATIC
  configfile/configfile.cpp)
target_include_directories(configfile PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# scanmatcher/
add_library(scanmatcher STATIC
  scanmatcher/smmap.cpp
  scanmatcher/scanmatcher.cpp
  scanmatcher/scanmatcherprocessor.cpp
  scanmatcher/eig3.cpp)
target_link_libraries(scanmatcher
  log sensor_range sensor_odometry sensor_base utils)
target_include_directories(scanmatcher PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# gridfastslam/
add_library(gridfastslam STATIC
  gridfastslam/gridslamprocessor_tree.cpp
  gridfastslam/motionmodel.cpp
  gridfastslam/gridslamprocessor.cpp
  gridfastslam/gfsreader.cpp)
target_link_libraries(gridfastslam
  scanmatcher log sensor_range sensor_odometry sensor_base utils)
target_include_directories(gridfastslam PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#############
## Install ##
#############

# 安装库和可执行文件
install(TARGETS 
  utils sensor_base sensor_odometry sensor_range 
  log configfile scanmatcher gridfastslam
  EXPORT openslam_gmapping-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 生成和安装导出头文件
generate_export_header(utils
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/utils/utils_export.h)
generate_export_header(sensor_base
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_base/sensor_base_export.h)
generate_export_header(sensor_odometry
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_odometry/sensor_odometry_export.h)
generate_export_header(sensor_range
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_range/sensor_range_export.h)
generate_export_header(log
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/log/log_export.h)
generate_export_header(configfile
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/configfile/configfile_export.h)
generate_export_header(scanmatcher
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/scanmatcher/scanmatcher_export.h)
generate_export_header(gridfastslam
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/gridfastslam/gridfastslam_export.h)

install(DIRECTORY ${EXPORT_HEADER_DIR}/
  DESTINATION include
)

# 生成配置文件
install(EXPORT openslam_gmapping-targets
  FILE openslam_gmappingTargets.cmake
  NAMESPACE openslam_gmapping::
  DESTINATION share/openslam_gmapping/cmake
)

ament_export_targets(openslam_gmapping-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp tf2 tf2_ros)

ament_package()
