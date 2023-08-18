file(GLOB_RECURSE sources "*.cpp")
file(GLOB headers "*.hpp")

add_library(rpc-friction-compensator SHARED ${sources} ${headers})

install(TARGETS rpc-friction-compensator DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/controller/friction_compensator")