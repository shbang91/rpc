FILE(GLOB headers "*.hh" "*.h" "*.hpp")
FILE(GLOB_RECURSE sources *.cc)
add_library (rpc-goldfarb SHARED ${headers} ${sources})

install(TARGETS rpc-goldfarb DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/optimizer/goldfarb")
