FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")
#FILE(GLOB sources2 "../alip_mpc/src/*.cpp")
#FILE(GLOB headers2 "../allip_mpc/include/*.hpp")

add_library(rpc-dcm-planner ${sources} ${headers}) #${sources2} ${headers2})
target_link_libraries(rpc-dcm-planner PUBLIC rpc-util rpc-alip-planner)

install(TARGETS rpc-dcm-planner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/planner/locomotion/dcm_planner")
