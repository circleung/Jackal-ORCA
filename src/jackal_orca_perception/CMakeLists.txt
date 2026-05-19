cmake_minimum_required(VERSION 3.8)
project(jackal_orca_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Python 스크립트를 실행 가능 위치(lib/<pkg>)에 설치
install(PROGRAMS
  scripts/depth_object_detector.py
  scripts/lidar_obstacle_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
