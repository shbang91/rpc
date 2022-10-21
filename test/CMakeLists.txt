#add_executable(interface_test interface_test.cpp)
#target_link_libraries(interface_test draco-pnc)

#add_executable(rolling_joint rolling_joint_test_main.cpp
    #rolling_joint_test_world_node.cpp)
#target_link_libraries(rolling_joint pnc-util
                                    #dart-robot-system
                                    #rj-module)

#add_executable(draco_dart_test draco_dart_robot_system_test.cpp)
#target_link_libraries(draco_dart_test rpc-dart-robot-system)

add_executable(pin_robot_system_test pin_robot_system_test.cpp)
target_link_libraries(pin_robot_system_test rpc-pin-robot-system)

add_executable(dcm_planner_test dcm_planner_test.cpp)
target_link_libraries(dcm_planner_test rpc-wbc)

add_executable(lmpc_handler_test lmpc_handler_test.cpp)
target_link_libraries(lmpc_handler_test rpc-lmpc-handler)
