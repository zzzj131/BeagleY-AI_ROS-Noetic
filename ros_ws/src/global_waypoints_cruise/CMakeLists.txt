cmake_minimum_required(VERSION 3.0.2)
project(global_waypoints_cruise) 

## 查找 Catkin 宏和依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  std_msgs
  sensor_msgs
  message_generation # 即使本包不生成消息，依赖 mavros_msgs 也需要它
  serial
)

# 查找所需的其他库
find_package(yaml-cpp REQUIRED)
find_library(MOSQUITTO_LIBRARY mosquitto REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp REQUIRED)

## 处理消息/服务/动作 (如果你的包自己定义了这些)
## 在这个例子中，我们只 *使用* mavros_msgs，所以不需要下面的 add_message_files 等
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
#
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
#
# generate_messages(
#   DEPENDENCIES
#   std_msgs mavros_msgs # 添加生成消息所需的其他消息包
# )

## Catkin 包配置
catkin_package(
#  INCLUDE_DIRS include        # 如果有 include 文件夹放头文件
#  LIBRARIES ${PROJECT_NAME}   # 如果构建了库
  CATKIN_DEPENDS roscpp mavros_msgs message_runtime std_msgs sensor_msgs# 运行时依赖
#  DEPENDS system_lib          # 非 Catkin 的系统依赖
)

## 指定包含目录
include_directories(
# 'include' # 如果有 include 文件夹
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
)

## 定义可执行文件
# 确保你的 C++ 文件放在 src/ 目录下，并命名为 global_waypoints_cruise.cpp
add_executable(global_waypoints_cruise_node src/global_waypoints_cruise.cpp)

## 添加编译时依赖 (如果可执行文件依赖于本包生成的消息/服务)
# add_dependencies(global_waypoints_cruise_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(global_waypoints_cruise_node ${catkin_EXPORTED_TARGETS}) # 通常这样就够了

## 链接库
target_link_libraries(global_waypoints_cruise_node
  ${catkin_LIBRARIES}
)

## 安装规则 
# 安装可执行文件
install(TARGETS global_waypoints_cruise_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # 或者用 ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安装 launch 文件 
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装 config 文件 
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# 如果有 include 文件夹，安装头文件
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# 编译航点加载器节点
add_executable(waypoints_loader_node src/waypoints_loader_node.cpp)
target_link_libraries(waypoints_loader_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${MOSQUITTO_LIBRARY}
  ${JSONCPP_LIBRARIES}
)

# 编译GPS记录器节点
add_executable(drone_gps_logger_node src/drone_gps_logger_node.cpp)
target_link_libraries(drone_gps_logger_node
  ${catkin_LIBRARIES}
  ${MOSQUITTO_LIBRARY}
  ${JSONCPP_LIBRARIES}
)