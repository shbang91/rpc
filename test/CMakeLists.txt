#googletest
find_package(GTest QUIET)
if (GTest_FOUND)
    message(STATUS "Using installed Google Test.")
    include_directories(${GTEST_INCLUDE_DIRS})
else()
    message(STATUS "Google TEST not found. Fetching from GitHub...")
    include(FetchContent)       # <-- Note: requires CMake version > 3.11
    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        release-1.11.0
            )
    FetchContent_MakeAvailable(googletest)
endif()
enable_testing()

add_executable(pin_robot_system_test pin_robot_system_test.cpp)
target_link_libraries(pin_robot_system_test rpc-pin-robot-system)

add_executable(dcm_planner_test dcm_planner_test.cpp)
target_link_libraries(dcm_planner_test rpc-wbc)

add_executable(draco_interface_test rpc_interface.cpp)
target_link_libraries(draco_interface_test rpc-draco-controller)

add_executable(ihwbc_test ihwbc_test.cpp)
target_link_libraries(ihwbc_test ${GTEST_LIBRARIES} rpc-draco-controller)

add_executable(friction_compensator_test karnopp_compensator_test.cpp)
target_link_libraries(friction_compensator_test gtest pthread rpc-friction-compensator)

add_executable(mujoco_pinocchio_comparison mujoco_pinocchio_comparison.cpp)
target_link_libraries(mujoco_pinocchio_comparison gtest pthread ${mujoco_LIBRARIES}
                                                        rpc-pin-robot-system)
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
