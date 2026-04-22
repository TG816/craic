cmake_minimum_required(VERSION 3.0.2)
project(flight_mission)

# 让这个包延迟编译，最后才编
set(CATKIN_ORDERED_BUILD ON)
set(CATKIN_LAST_PKG ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  livox_ros_driver
  visualization_msgs
  #quadrotor_msgs
  cloud_recognition
  ring_detector
)

find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc imgcodecs objdetect dnn)

# ==============================================
# 自动查找 ONNX Runtime（不管装在哪都能找到）
# ==============================================
find_path(ONNX_INCLUDE_DIR
  NAMES onnxruntime_c_api.h
  PATHS /usr/local /opt /usr /home
  PATH_SUFFIXES onnxruntime/include include
)

find_library(ONNX_LIBRARY
  NAMES onnxruntime
  PATHS /usr/local /opt /usr /home
  PATH_SUFFIXES onnxruntime/lib lib
)

# ==============================================
# 自动查找 QR 码库 quirc（自动找，不用写路径）
# ==============================================
find_library(QR_LIBRARY
  NAMES quirc
  PATHS /usr/local /opt /usr
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_recognition ring_detector
)

# 头文件包含
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ONNX_INCLUDE_DIR}
)

# ======================== 这里开始我帮你改成库 ========================
# 1. 把所有公共文件编译成库（只编一次）
add_library(mission_common
  src/mission_globals.cpp
  src/mission_callbacks.cpp
  src/flight_control.cpp
  src/obstacle_avoidance.cpp
  src/ring_detection.cpp
  src/vision_detection.cpp
  src/mission_tools.cpp
  src/map_class.cpp
)

# 库链接所有依赖
target_link_libraries(mission_common
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${ONNX_LIBRARY}
  ${QR_LIBRARY}
)

# 2. 飞行主程序（只需要 main.cpp）
add_executable(flight_mission 
  src/main.cpp
)
target_link_libraries(flight_mission 
  mission_common  # 链接公共库
)

# 3. YOLO 测试程序（只需要 yolo.cpp）
add_executable(yolo 
  src/m_yolo.cpp
)
target_link_libraries(yolo 
  mission_common  # 链接同一个库！
)

# 4.简化版 绕桩加穿环
add_executable(circleAndring 
  src/m_circle_ring_task.cpp
)
target_link_libraries(circleAndring 
  mission_common  # 链接同一个库！
)

# 5. test_circle测试程序
add_executable(test_circle 
  src/test_circle.cpp
)
target_link_libraries(test_circle 
  mission_common  # 链接同一个库！
)

# 6. test_ring 测试程序
add_executable(test_ring
  src/test_ring.cpp
)
target_link_libraries(test_ring 
  mission_common  # 链接同一个库！
)

# ====================================================================



# 运行时自动找到 ONNX 库（不报错）
get_filename_component(ONNX_LIB_DIR ${ONNX_LIBRARY} DIRECTORY)
set_target_properties(flight_mission PROPERTIES
  INSTALL_RPATH ${ONNX_LIB_DIR}
  BUILD_WITH_INSTALL_RPATH ON
)