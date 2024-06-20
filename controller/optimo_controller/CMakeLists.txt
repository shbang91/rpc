file(GLOB_RECURSE sources "*.cpp")
file(GLOB controller_headers "*.hpp")

add_library(rpc-optimo-controller SHARED ${sources} ${controller_headers})

target_link_libraries(rpc-optimo-controller PUBLIC rpc-pin-robot-system
                                            PUBLIC rpc-wbc
                                            PUBLIC rpc-util)
