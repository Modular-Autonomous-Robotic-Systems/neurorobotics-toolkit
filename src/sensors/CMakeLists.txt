cmake_minimum_required(VERSION 3.14)
project(sensors)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Command to check gstreamer dependencies
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0  gstreamer-app-1.0)

link_directories( ${GST_LIBRARY_DIRS}  )
include_directories(
	include
	$<INSTALL_INTERFACE:include>
	${OpenCV_INCLUDE_DIRS}
	${GST_INCLUDE_DIRS}) # include headers for opencv in all executables

set(THIS_PACKAGE_INCLUDE_DEPENDS
	rclcpp
	sensor_msgs
	cv_bridge
	image_transport
	std_msgs
	rclcpp_lifecycle
	lifecycle_msgs
)

add_executable(video_logger_node
	src/video/logger_main.cpp
	src/video/logger.cpp
	src/video/common.cpp
)
add_executable(video_reader_node
	src/video/reader_main.cpp
	src/video/reader.cpp
	src/video/common.cpp
)

add_executable(imu_logger_node
	src/imu/imu_subscriber.cpp
)

ament_target_dependencies(video_logger_node
	PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(video_reader_node
	PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(imu_logger_node
	PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(video_logger_node
	PUBLIC ${OpenCV_LIBS}
	PUBLIC ${GST_LIBRARIES}
)
target_link_libraries(video_reader_node
	PUBLIC ${OpenCV_LIBS}
	PUBLIC ${GST_LIBRARIES}
)

install (DIRECTORY include/ 
	DESTINATION include
)

install(TARGETS video_logger_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
	INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(TARGETS video_reader_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
	INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Install executable and necessary files
install(TARGETS imu_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
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

ament_package()
