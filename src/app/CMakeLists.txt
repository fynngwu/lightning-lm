if (LIGHTNING_WITH_UI)
    add_executable(run_frontend_offline
            run_frontend_offline.cc
    )

    target_link_libraries(run_frontend_offline
            lightning_system_core
    )

    add_executable(run_loop_offline
            run_loop_offline.cc
    )

    target_link_libraries(run_loop_offline
            lightning_system_core
    )
endif ()

add_executable(run_slam_offline
        run_slam_offline.cc
)

target_link_libraries(run_slam_offline
        lightning_system_core
)

add_executable(run_slam_online
        run_slam_online.cc
)

target_link_libraries(run_slam_online
        lightning_system_core
)

add_executable(run_loc_offline
        run_loc_offline.cc
)

target_link_libraries(run_loc_offline
        lightning_loc_core
        lightning_system_core
)

add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        lightning_system_core
)

if (LIGHTNING_WITH_UI)
    add_executable(test_ui
            test_ui.cc
    )

    target_link_libraries(test_ui
            lightning_system_core
    )
endif ()

install(TARGETS run_slam_offline run_slam_online run_loc_offline run_loc_online
        DESTINATION lib/${PROJECT_NAME})
