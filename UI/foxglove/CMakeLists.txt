add_subdirectory(client)

add_executable(simple-client-subscriber simple_client_subscriber.cpp)
target_link_libraries(simple-client-subscriber
        foxglove-websocket::foxglove-websocket
        Boost::program_options
)

install(TARGETS simple-client-subscriber DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/ui/simple_client_subscriber")