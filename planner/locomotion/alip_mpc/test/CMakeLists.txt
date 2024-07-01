#cmake_minimum_required(VERSION 3.1...3.27)
#project(Test_Alip_MPC_v2)

#set(CMAKE_CXX_STANDARD 17)


#find_package(casadi REQUIRED)


#add_library(ctrl_lib ${CMAKE_CURRENT_SOURCE_DIR}/../src/NewStep_mpc.cpp)
#target_include_directories(ctrl_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
#target_link_libraries(ctrl_lib casadi)


add_library(test_lib test_mpc_dyn.cpp test_mpc_dyn.hpp)


add_executable(TEST0_alip_mpc test_mpc_dyn.cpp)
target_link_libraries(TEST0_alip_mpc test_lib rpc-alip-planner rpc-util)


