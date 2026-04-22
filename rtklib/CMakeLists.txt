project(rtklib)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
        )
find_package(Gflags)
find_package(Glog)
find_package(Ceres REQUIRED)
# find_package(OpenCV REQUIRED QUIET)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        # ${OpenCV_INCLUDE_DIRS}

)

add_message_files(
        DIRECTORY msg
        FILES
        cloud_info.msg
        feature_info.msg
        obsdt.msg
        sat_state.msg
        satdt.msg
        GNSS_Info_ZD.msg
        GNSS_Info_SD.msg
        GNSS_Info.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
)

        
catkin_package(
    LIBRARIES rtklib  # 导出 rtklib 库
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    message_generation
    visualization_msgs
)
aux_source_directory(./src RTKLIB_SRCS)
aux_source_directory(./src/rcv RTKLIB_RCV)
add_library(rtklib STATIC ${RTKLIB_SRCS} ${RTKLIB_RCV})
add_dependencies(rtklib ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(rtklib PRIVATE include)
target_link_libraries(rtklib libm.so libpthread.so blas lapack gflags ${catkin_LIBRARIES})