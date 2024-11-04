cmake_minimum_required(VERSION 3.8)
project(face_age_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# setting variables
set(SYS_ROOT ${CMAKE_SYSROOT})
message("=> SYS_ROOT is " ${SYS_ROOT})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
include_directories(${SYS_ROOT}/usr/include)
include_directories(${SYS_ROOT}/usr/hobot/include)
link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu
  ${SYS_ROOT}/usr/hobot/lib
  ${SYS_ROOT}/usr/lib
)

# add executable
add_executable(face_age_detection src/main.cpp src/face_age_det_node.cpp src/face_age_det_output_parser.cpp src/ai_msg_manage.cpp src/img_convert_utils.cpp)
target_compile_features(face_age_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(face_age_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(face_age_detection
  dnn
)
ament_target_dependencies(face_age_detection
  rclcpp
  OpenCV
  sensor_msgs
  dnn_node
  ai_msgs
  hbm_img_msgs
)

# install
install(TARGETS face_age_detection DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
