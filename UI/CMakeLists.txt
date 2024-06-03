if(BUILD_WITH_FOXGLOVE)
    set(B_USE_FOXGLOVE true)

    find_package(foxglove-schemas-protobuf REQUIRED)
    find_package(foxglove-websocket REQUIRED)
    find_package(Boost COMPONENTS program_options REQUIRED)

    # warning message if trying to build with foxglove but without zmq/protobuf
    if (NOT BUILD_WITH_ZMQ_PROTOBUF)
        message("\n*** WARNING ***\nPLEASE ENABLE ZMQ/PROTOBUF -- needed dependencies\n")
    endif ()
else()
    set(B_USE_FOXGLOVE false)
endif()
