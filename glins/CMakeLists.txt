cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(glins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        cv_bridge
        # pcl library
        pcl_conversions
        # msgs
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        rosbag
        rosbag_storage
        rtklib
        )
find_package(Gflags)
find_package(Glog)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
        DIRECTORY msg
        FILES
        cloud_info.msg
        obsdt.msg
        sat_state.msg
        satdt.msg
        GNSS_Info_ZD.msg
        GNSS_Info_SD.msg
        GNSS_Info.msg
        feature_info.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        save_map.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
)

# add_subdirectory(./ThirdParty/rtklib/)

set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
# include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        # ./ThirdParty/rtklib/include/
)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
# link directories
link_directories(
        include
        ${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBRARY_DIRS}
        ${GTSAM_LIBRARY_DIRS}
        ${CERES_LIBRARIES}
)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL GTSAM

        CATKIN_DEPENDS
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        message_runtime
        message_generation
        visualization_msgs
        rtklib
)
###########
## Build ##
###########

add_executable(${PROJECT_NAME}_test_rtk src/test_rtk.cpp)
add_dependencies(${PROJECT_NAME}_test_rtk ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_test_rtk ${catkin_LIBRARIES} gflags gtsam dw)

add_executable(${PROJECT_NAME}_test_gins src/test_gins.cpp)
add_dependencies(${PROJECT_NAME}_test_gins ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(${PROJECT_NAME}_test_gins PUBLIC include)
target_link_libraries(${PROJECT_NAME}_test_gins ${catkin_LIBRARIES} gflags gtsam dw)

file(GLOB lio_files
        "include/imu/imuPreintegration.cpp"
        "include/mapping/mapOptmizationGps.cpp"
        "include/lidar/featureExtraction.cpp"
        "include/lidar/imageProjection.cpp")

add_library(lio ${lio_files})
add_dependencies(lio ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_include_directories(lio PUBLIC include)
target_link_libraries(lio ${catkin_LIBRARIES} gflags gtsam OpenMP::OpenMP_CXX dw ${CERES_LIBRARIES})
# target_precompile_headers(lio PRIVATE ./ThirdParty/rtklib/include/gnss_tools.h)
add_executable(${PROJECT_NAME}_test_lio src/test_lio.cpp)
add_dependencies(${PROJECT_NAME}_test_lio ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_test_lio ${catkin_LIBRARIES}  gflags gtsam lio dw)


