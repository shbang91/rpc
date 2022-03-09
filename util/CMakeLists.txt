FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB_RECURSE headers "*.hpp")

add_library(pnc_util ${sources})
target_link_libraries(pnc_util pnc_yaml)

install(TARGETS pnc_util DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/util")
