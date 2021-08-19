cmake_minimum_required(VERSION 3.0.2)
project(draft_pkg)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_compile_options(-std=c++11 -O3 -g -DNDEBUG -pthread)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog  REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include  
#  LIBRARIES draft_pkg
#  CATKIN_DEPENDS cv_bridge geometry_msgs pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS})
add_executable(process_ifm
src/process_ifm.cpp
)
target_link_libraries(process_ifm
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    gflags
    ${GLOG_LIBRARY}
)

add_executable(pcl_icp src/pcl_icp.cpp)
target_link_libraries(pcl_icp
${Boost_LIBRARIES}
${OpenCV_LIBS}
${PCL_LIBRARIES}
)

add_executable(boost_eigen_test src/boost_eigen_test.cpp)
target_link_libraries(boost_eigen_test
${Boost_LIBRARIES}
${OpenCV_LIBS}
${PCL_LIBRARIES}
gflags
${GLOG_LIBRARY}
)

add_executable(pcl_ransac_line src/pcl_ransac_line.cpp)
target_link_libraries(pcl_ransac_line
${Boost_LIBRARIES}
${OpenCV_LIBS}
${PCL_LIBRARIES}
)

add_executable(ros_test src/ros_test.cpp)
target_link_libraries(ros_test
gflags
${GLOG_LIBRARY}
${catkin_LIBRARIES}
)

add_executable(point_cloud_trans src/point_cloud_trans.cpp)
target_link_libraries(point_cloud_trans
gflags
${GLOG_LIBRARY}
${catkin_LIBRARIES}
)

add_executable(ros_imu_angle src/ros_imu_angle.cpp)
target_link_libraries(ros_imu_angle
gflags
${GLOG_LIBRARY}
${catkin_LIBRARIES}
)

add_executable(opencv_cvui src/opencv_cvui.cpp)
target_link_libraries(opencv_cvui
gflags
${GLOG_LIBRARY}
${OpenCV_LIBS}
)

add_executable(read_image_to_ros src/read_image_to_ros.cpp)
target_link_libraries(read_image_to_ros
gflags
${GLOG_LIBRARY}
${catkin_LIBRARIES}
${OpenCV_LIBS}
)

add_executable(kitti_bin2pcd src/kitti/kitti_bin2pcd.cpp)
target_link_libraries(kitti_bin2pcd
gflags
${GLOG_LIBRARY}
${Boost_LIBRARIES}
${PCL_LIBRARIES}
boost_program_options
)
add_executable(mvg_codes src/MVG_codes.cpp)
target_link_libraries(mvg_codes
${Boost_LIBRARIES}
${OpenCV_LIBS}
${PCL_LIBRARIES}
gflags
${GLOG_LIBRARY}
)


install(DIRECTORY  scripts include image launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)

install(TARGETS
    process_ifm pcl_icp boost_eigen_test pcl_ransac_line
    ros_test  opencv_cvui read_image_to_ros kitti_bin2pcd
    ros_imu_angle point_cloud_trans mvg_codes
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_draft_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
