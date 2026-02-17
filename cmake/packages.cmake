find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search registration)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if (LIGHTNING_WITH_UI)
    find_package(Pangolin CONFIG REQUIRED)
    if (Pangolin_DIR MATCHES "^${PROJECT_SOURCE_DIR}/thirdparty")
        message(FATAL_ERROR
                "Pangolin is resolved from repository thirdparty path (${Pangolin_DIR}). "
                "Install and use system package instead (e.g. apt ros-humble-pangolin / libpangolin-dev).")
    endif ()
    message(STATUS "Using system Pangolin from: ${Pangolin_DIR}")
    find_package(OpenGL REQUIRED)
endif ()

# OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

if (BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
else ()
    add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif ()

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
)

# Pangolin include paths come from imported targets in PangolinConfig.cmake.

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(third_party_libs
        glog gflags
        ${yaml-cpp_LIBRARIES}
        tbb
)

if (LIGHTNING_WITH_UI)
    list(APPEND third_party_libs ${Pangolin_LIBRARIES})
endif ()

set(lightning_pcl_libs ${PCL_LIBRARIES})
set(lightning_cv_libs ${OpenCV_LIBS})
set(lightning_rosbag_libs ${rosbag2_cpp_LIBRARIES})
