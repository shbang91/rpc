add_executable(pin_robot_system_test pin_robot_system_test.cpp)
target_link_libraries(pin_robot_system_test rpc-pin-robot-system)

add_executable(dcm_planner_test dcm_planner_test.cpp)
target_link_libraries(dcm_planner_test rpc-wbc)

add_executable(lmpc_handler_test lmpc_handler_test.cpp)
target_link_libraries(lmpc_handler_test rpc-lmpc-handler)

add_executable(draco_interface_test rpc_interface.cpp)
target_link_libraries(draco_interface_test rpc-draco-controller)

if(BUILD_WITH_MATLOGGER)
    add_executable(min_jerk_traj_test min_jerk_traj.cpp)
    target_link_libraries(min_jerk_traj_test matlogger2::matlogger2)
    target_link_libraries(min_jerk_traj_test rpc-util)
endif()

if(BUILD_WITH_DART)
    add_subdirectory(rolling_joint)
    add_executable(rolling_joint_test rolling_joint_test_main.cpp
        rolling_joint_test_world_node.cpp)
    target_link_libraries(rolling_joint_test rpc-util
                                        rpc-dart-robot-system
                                        rj-module)

    add_executable(draco_dart_test draco_dart_robot_system_test.cpp)
    target_link_libraries(draco_dart_test rpc-dart-robot-system)
endif()


