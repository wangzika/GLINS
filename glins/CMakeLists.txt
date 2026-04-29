cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)

# 工程名，对应 catkin 包 glins
project(glins)

# C++ 标准与编译模式
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# 查找 catkin 组件依赖
find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        cv_bridge
        jsk_recognition_msgs
        # pcl library
        pcl_conversions
        # msgs
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        jsk_recognition_msgs
        message_generation
        visualization_msgs
        rosbag
        rosbag_storage
        rtklib
        )

# 查找第三方库依赖
find_package(Gflags)
find_package(Glog)
find_package(Ceres REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

# 注册本包自定义消息
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

# 注册本包自定义服务
add_service_files(
        DIRECTORY srv
        FILES
        save_map.srv
)

# 生成消息/服务代码，并声明这些消息依赖的标准消息包
generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
)

# add_subdirectory(./ThirdParty/rtklib/)

set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})

# 头文件搜索路径
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

# 链接库搜索路径
link_directories(
        include
        ${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBRARY_DIRS}
        ${GTSAM_LIBRARY_DIRS}
        ${CERES_LIBRARIES}
)

# 导出给其他 catkin 包使用的包含目录、系统依赖和 catkin 依赖
catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL GTSAM

        CATKIN_DEPENDS
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
        message_runtime
        message_generation
        visualization_msgs
        rtklib
)
###########
## Build ##
###########

# 平台相关库集合：
# 例如 OpenMP、Linux 下的 libdw，后面按平台追加。
set(GLINS_PLATFORM_LIBS)

if (OpenMP_CXX_FOUND)
    list(APPEND GLINS_PLATFORM_LIBS OpenMP::OpenMP_CXX)
endif()

# 仅在 Linux 下尝试链接 libdw（栈回溯 / 调试相关）
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_library(DW_LIBRARY dw)
    if (DW_LIBRARY)
        list(APPEND GLINS_PLATFORM_LIBS ${DW_LIBRARY})
    endif()
endif()

# RTKLIB 测试程序
add_executable(${PROJECT_NAME}_test_rtk src/test_rtk.cpp)
add_dependencies(${PROJECT_NAME}_test_rtk ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_test_rtk ${catkin_LIBRARIES} gflags gtsam ${GLINS_PLATFORM_LIBS})

# GINS 主测试程序（GNSS + IMU 融合链）
add_executable(${PROJECT_NAME}_test_gins src/test_gins.cpp)
add_dependencies(${PROJECT_NAME}_test_gins ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(${PROJECT_NAME}_test_gins PUBLIC include)
target_link_libraries(${PROJECT_NAME}_test_gins ${catkin_LIBRARIES} gflags gtsam ${GLINS_PLATFORM_LIBS})

# LIO 主功能源码，收拢成一个单独库，供 test_lio 复用
file(GLOB lio_files
        "include/imu/imuPreintegration.cpp"
        "include/mapping/mapOptmizationGps.cpp"
        "include/lidar/featureExtraction.cpp"
        "include/lidar/imageProjection.cpp")

add_library(lio ${lio_files})
add_dependencies(lio ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_include_directories(lio PUBLIC include)
target_link_libraries(lio ${catkin_LIBRARIES} gflags gtsam ${GLINS_PLATFORM_LIBS} ${CERES_LIBRARIES})
# target_precompile_headers(lio PRIVATE ./ThirdParty/rtklib/include/gnss_tools.h)

# LIO 离线测试程序，链接上面封装好的 lio 库
add_executable(${PROJECT_NAME}_test_lio src/test_lio.cpp)
add_dependencies(${PROJECT_NAME}_test_lio ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_test_lio ${catkin_LIBRARIES}  gflags gtsam lio ${GLINS_PLATFORM_LIBS})
