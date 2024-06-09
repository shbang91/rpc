FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB headers *.hpp)

add_library(foxglove-parameter-subscriber ${sources} ${headers})
target_link_libraries(foxglove-parameter-subscriber
        foxglove-websocket::foxglove-websocket
        Boost::program_options
)

install(TARGETS foxglove-parameter-subscriber DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/ui/foxglove_parameter_subscriber")