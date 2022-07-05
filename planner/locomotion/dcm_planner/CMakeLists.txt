FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(dcm_planner SHARED ${sources} ${headers})
target_link_libraries(dcm_planner rpc-util)

install(TARGETS dcm_planner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/planner/locomotion/dcm_planner")
