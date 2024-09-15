add_subdirectory(dcm_planner)

if (BUILD_WITH_HPIPM)
    add_subdirectory(convex_mpc)
endif()
