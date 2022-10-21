FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(rpc-dcm-planner ${sources} ${headers})
target_link_libraries(rpc-dcm-planner PUBLIC rpc-util)

install(TARGETS rpc-dcm-planner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/planner/locomotion/dcm_planner")
