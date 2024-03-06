FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(rpc-model ${sources} ${headers})

install(TARGETS rpc-model DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/controller/models")
