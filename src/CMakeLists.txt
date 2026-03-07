add_library(lightning_lio_core SHARED
        common/nav_state.cc
        common/options.cc
        common/params.cc
        io/yaml_io.cc
        io/file_io.cc
        utils/timer.cc
        utils/pointcloud_utils.cc
        core/lio/eskf.cc
        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc
)

ament_target_dependencies(
        lightning_lio_core
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "std_srvs"
        "message_filters"
        "pcl_conversions"
        "tf2_ros"
        "tf2"
        "livox_ros_driver2"
)

target_link_libraries(lightning_lio_core
        ${third_party_libs}
        ${lightning_pcl_libs}
        ${lightning_cv_libs}
)

add_library(lightning_map_core SHARED
        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc
        core/g2p5/g2p5_subgrid.cc
        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc
)

ament_target_dependencies(
        lightning_map_core
        "sensor_msgs"
        "nav_msgs"
)

target_link_libraries(lightning_map_core
        lightning_lio_core
        ${third_party_libs}
        ${lightning_pcl_libs}
        ${lightning_cv_libs}
)

include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
add_subdirectory(core/miao)

add_library(lightning_loop_core SHARED
        core/loop_closing/loop_closing.cc
        core/localization/localization_result.cc
        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc
)

ament_target_dependencies(
        lightning_loop_core
        "sensor_msgs"
        "nav_msgs"
)

target_link_libraries(lightning_loop_core
        lightning_lio_core
        miao.core
        miao.utils
        ${third_party_libs}
        ${lightning_pcl_libs}
        ${lightning_cv_libs}
)

add_library(lightning_loc_core SHARED
        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/pclomp/ndt_omp.cpp
        core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp
        core/localization/localization.cpp
)

ament_target_dependencies(
        lightning_loc_core
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "tf2_ros"
        "tf2"
        "livox_ros_driver2"
)

target_link_libraries(lightning_loc_core
        lightning_lio_core
        lightning_map_core
        lightning_loop_core
        ${third_party_libs}
        ${lightning_pcl_libs}
        ${lightning_cv_libs}
)

add_library(lightning_system_core SHARED
        core/system/slam.cc
        core/system/loc_system.cc
        ui/publish_window.cc
        wrapper/bag_io.cc
)

ament_target_dependencies(
        lightning_system_core
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "std_srvs"
        "message_filters"
        "pcl_conversions"
        "tf2_ros"
        "tf2"
        "livox_ros_driver2"
)

add_dependencies(lightning_system_core
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

target_link_libraries(lightning_system_core
        lightning_lio_core
        lightning_map_core
        lightning_loop_core
        lightning_loc_core
        ${third_party_libs}
        ${lightning_rosbag_libs}
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

if (LIGHTNING_WITH_UI)
    add_library(lightning_ui_core SHARED
            ui/pangolin_window.cc
            ui/pangolin_window_impl.cc
            ui/ui_car.cc
            ui/ui_cloud.cc
            ui/ui_trajectory.cc
    )

    ament_target_dependencies(
            lightning_ui_core
            "rclcpp"
            "std_msgs"
            "geometry_msgs"
            "sensor_msgs"
            "nav_msgs"
            "tf2_ros"
            "tf2"
    )

    target_link_libraries(lightning_ui_core
            ${third_party_libs}
            ${lightning_cv_libs}
    )

    target_link_libraries(lightning_lio_core lightning_ui_core)
    target_link_libraries(lightning_loc_core lightning_ui_core)
    target_link_libraries(lightning_system_core lightning_ui_core)
endif ()

# 兼容层：保留原始聚合库名，便于平滑迁移。
add_library(${PROJECT_NAME}.libs INTERFACE)

target_link_libraries(${PROJECT_NAME}.libs
        INTERFACE
        lightning_system_core
        lightning_loc_core
        lightning_loop_core
        lightning_map_core
        lightning_lio_core
)

add_dependencies(${PROJECT_NAME}.libs
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

set(lightning_install_targets
        lightning_lio_core
        lightning_map_core
        lightning_loop_core
        lightning_loc_core
        lightning_system_core
        miao.core
        miao.utils
)

if (TARGET lightning_ui_core)
    list(APPEND lightning_install_targets lightning_ui_core)
endif ()

install(TARGETS ${lightning_install_targets}
        DESTINATION lib)

add_subdirectory(app)
