FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(rpc-state-estimator ${sources} ${headers})

target_link_libraries(rpc-state-estimator rpc-util)

install(TARGETS rpc-state-estimator DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/rpc/state_estimator")
