cmake_minimum_required(VERSION 3.0.2)
project(rm_vision_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加对catkin的依赖
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  roscpp
  rospy
  rm_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  message_generation
  dynamic_reconfigure
)

# 如果使用C++代码，添加这些依赖
# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)

catkin_python_setup()

# 生成消息、服务和动作（如果需要）
################################################
## Declare ROS messages, services and actions ##
################################################

# 如果需要自定义消息，取消注释以下内容
# add_message_files(
#   FILES
#   DetectionResult.msg
#   InspectionCommand.msg
# )

# 如果需要自定义服务，取消注释以下内容
# add_service_files(
#   FILES
#   StartDetection.srv
#   GetDetectionResults.srv
# )

# 如果需要自定义动作，取消注释以下内容
# add_action_files(
#   FILES
#   ExecuteInspection.action
# )

# 生成消息、服务和动作
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   sensor_msgs
#   std_msgs
#   rm_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES rm_vision_control
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    std_msgs
    roscpp
    rospy
    rm_msgs
    tf
    tf2_ros
    cv_bridge
    image_transport
  #  DEPENDS system_lib
)

###########
## Build ##
###########

# 包含目录
include_directories(
  # ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 安装Python脚本
catkin_install_python(PROGRAMS
  src/rm_vision_control/main_controller.py
  src/rm_vision_control/arm_controller.py
  src/rm_vision_control/vision_detector.py
  src/rm_vision_control/realsense_processor.py
  src/rm_vision_control/crack_detection.py
  src/rm_vision_control/point_inspection.py
  src/rm_vision_control/utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

# 安装Python模块
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "*.py" PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

# 如果需要编译C++代码，取消注释以下部分
############################################
## C++ 代码编译部分（如果以后需要）##
############################################

# # 添加可执行文件
# add_executable(arm_control_node
#   src/arm_control_node.cpp
#   src/arm_controller.cpp
# )
# 
# # 添加依赖
# add_dependencies(arm_control_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# 
# # 链接库
# target_link_libraries(arm_control_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
# 
# # 添加视觉处理节点
# add_executable(vision_processor_node
#   src/vision_processor_node.cpp
#   src/vision_processor.cpp
#   src/yolo_detector.cpp
# )
# 
# add_dependencies(vision_processor_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# 
# target_link_libraries(vision_processor_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
# 
# # 安装可执行文件
# install(TARGETS arm_control_node vision_processor_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

# 启用测试
if(CATKIN_ENABLE_TESTING)
  # 如果需要C++测试，取消注释
  # find_package(rostest REQUIRED)
  # add_rostest(test/test_arm_control.test)
  # add_rostest(test/test_vision_integration.test)
endif()

#############
## Install ##
#############

# 安装其他文件
install(FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
