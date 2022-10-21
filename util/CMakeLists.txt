FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB_RECURSE headers "*.hpp")

add_library(rpc-util ${sources} ${headers})
target_link_libraries(rpc-util PUBLIC rpc-yaml)

install(TARGETS rpc-util DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/util")
