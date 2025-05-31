cmake_minimum_required(VERSION 3.8)
project(controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ardupilot_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
	rclcpp
	sensor_msgs
	std_msgs
	tello_msgs
	lifecycle_msgs
	rclcpp_lifecycle
)


add_executable(tello_controller_node
	controllers/tello_controller/tello_controller.cpp
	controllers/tello_controller/tello_controller_main.cpp
)
ament_target_dependencies(tello_controller_node
	PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(video_logging_controller_node
	src/logging/video_controller.cpp
	src/common/controller.cpp
)
ament_target_dependencies(video_logging_controller_node
	PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
	PUBLIC ardupilot_msgs
)
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

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
	scripts/orbslam3_monocular_controller
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
install(TARGETS 
	tello_controller_node
	video_logging_controller_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)
install (DIRECTORY include/ 
	DESTINATION include
)
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
