cmake_minimum_required(VERSION 3.0.2)
project(traj_lo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  	geometry_msgs
  	nav_msgs
  	sensor_msgs
  	roscpp
  	rospy
  	std_msgs
  	pcl_ros
  	tf
  	livox_ros_driver
  	message_generation
  	eigen_conversions
)

include_directories(${rosbag_INCLUDE_DIRS})
link_libraries(${rosbag_LIBRARIES})

find_package(OpenCV REQUIRED
	COMPONENTS
	opencv_highgui
	CONFIG
)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)
pkg_check_modules(ncurses REQUIRED ncurses)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

catkin_package(
  	CATKIN_DEPENDS
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${JSONCPP_INCLUDE_DIRS}
)

# add_executable(traj_lo src/entrance_tracking.cpp ${POSE_TRACKING_FILE})
# target_link_libraries(traj_lo yaml-cpp ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_subdirectory(thirdparty)

include_directories(include)

add_definitions("-D SOPHUS_USE_BASIC_LOGGING")
include_directories(thirdparty/Sophus)

add_executable(traj_lo src/entrance_tracking.cpp
        src/io/data_loader.cpp
        src/utils/config.cpp
        src/core/odometry.cpp
        src/core/map_manager.cpp
)

target_link_libraries(traj_lo PUBLIC 
		yaml-cpp ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
        ${TBB_LIBRARIES}
        pthread
        PRIVATE tsl::robin_map
        TBB::tbb
)