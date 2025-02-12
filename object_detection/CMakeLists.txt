cmake_minimum_required(VERSION 3.5)
project(object_detection)

# C++ 표준 (필요시 14 이상으로 설정)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ament_cmake 및 의존 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# List the message files
#set(msg_files
#  "msg/DamviDetection.msg"
#  "msg/DamviOpponentOdom.msg"
#)

# Generate message files
#rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
#  DEPENDENCIES std_msgs nav_msgs  # Add other dependencies if needed
#)


# scan_processor_node: 레이저 스캔 전처리 및 장애물 후보 생성
add_executable(scan_processor_node src/scan_processor_node.cpp)
ament_target_dependencies(scan_processor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  #rosidl_default_runtime
)
#rosidl_target_interfaces(scan_processor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# obstacle_tracker_node: 장애물 후보를 받아 추적 수행
add_executable(obstacle_tracker_node src/obstacle_tracker_node.cpp)
ament_target_dependencies(obstacle_tracker_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  #rosidl_default_runtime
)
#rosidl_target_interfaces(obstacle_tracker_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# visualization_node: 추적 결과를 MarkerArray로 시각화
add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node
  rclcpp
  visualization_msgs
)

# occupancy_grid_node: pgm 파일 사용해 map 토픽 발행
add_executable(occupancy_grid_node src/occupancy_grid_node.cpp)
ament_target_dependencies(occupancy_grid_node
  rclcpp
  nav_msgs
)


# 설치 대상 설정
install(TARGETS
  scan_processor_node
  obstacle_tracker_node
  visualization_node
  occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일이나 기타 리소스 디렉토리도 설치할 경우
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
