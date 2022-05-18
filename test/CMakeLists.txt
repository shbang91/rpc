#add_executable(interface_test interface_test.cpp)
#target_link_libraries(interface_test draco-pnc)

add_executable(rolling_joint rolling_joint_test_main.cpp
    rolling_joint_test_world_node.cpp)
target_link_libraries(rolling_joint pnc-util
                                    dart-robot-system
                                    rj-module)
add_executable(robot_system_test robot_system_test.cpp)
target_link_libraries(robot_system_test dart-robot-system
                                        pinocchio-robot-system)
