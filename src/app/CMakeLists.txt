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

if (LIGHTNING_BUILD_LOC_APPS)
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
endif ()

if (LIGHTNING_WITH_UI)
    add_executable(test_ui
            test_ui.cc
    )

    target_link_libraries(test_ui
            lightning_system_core
    )
endif ()

set(LIGHTNING_APP_TARGETS
        run_slam_offline
        run_slam_online
)

if (LIGHTNING_BUILD_LOC_APPS)
    list(APPEND LIGHTNING_APP_TARGETS
            run_loc_offline
            run_loc_online
    )
endif ()

install(TARGETS ${LIGHTNING_APP_TARGETS}
        DESTINATION lib/${PROJECT_NAME})
