FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(rpc-filters ${sources})
target_link_libraries(rpc-filters rpc-util)

install(TARGETS rpc-filters DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/filters")
