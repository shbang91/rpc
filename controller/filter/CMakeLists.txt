FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(rpc-filter ${sources})
target_link_libraries(rpc-filter PUBLIC rpc-util)

install(TARGETS rpc-filter DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/filter")
