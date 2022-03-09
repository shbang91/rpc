add_executable(interface_test interface_test.cpp)
target_link_libraries(interface_test draco-pnc)

add_executable(rolling_joint rolling_joint_test_main.cpp
    rolling_joint_test_world_node.cpp)
target_link_libraries(rolling_joint ${DART_LIBRARIES}
                                    pnc_util
                                    pnc_yaml
                                    robot-system
                                    rj-module)
