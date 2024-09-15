file(GLOB simulate_sources "simulate/*.cc" "simulate/*.cpp")
file(GLOB simulate_headers "simulate/*.h")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(run_optimo main.cc ${simulate_sources} ${simulate_headers})
target_link_libraries(run_optimo ${mujoco_LIBRARIES}
                                ${glfw3_LIBRARIES}
                                glfw
                                Threads::Threads)
target_link_libraries(run_optimo rpc-optimo-controller)
