file(GLOB simulate_sources "simulate/*.cc" "simulate/*.cpp")
file(GLOB simulate_headers "simulate/*.h")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(run_draco main.cc ${simulate_sources} ${simulate_headers})
target_link_libraries(run_draco ${mujoco_LIBRARIES}
                                ${glfw3_LIBRARIES}
                                glfw
                                Threads::Threads)
target_link_libraries(run_draco rpc-draco-controller)

add_executable(run_draco_manipulation main_manipulation.cc ${simulate_sources} ${simulate_headers})
target_link_libraries(run_draco_manipulation ${mujoco_LIBRARIES}
                                ${glfw3_LIBRARIES}
                                glfw
                                Threads::Threads)
target_link_libraries(run_draco_manipulation rpc-draco-controller)

add_executable(sake_ezgripper sake_ezgripper.cc ${simulate_sources} ${simulate_headers})
target_link_libraries(sake_ezgripper ${mujoco_LIBRARIES}
                                ${glfw3_LIBRARIES}
                                glfw
                                Threads::Threads)
