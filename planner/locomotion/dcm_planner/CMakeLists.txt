FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(rpc-dcm_planner SHARED ${sources} ${headers})
target_link_libraries(rpc-dcm_planner rpc-util)

install(TARGETS rpc-dcm_planner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/planner/locomotion/dcm_planner")
