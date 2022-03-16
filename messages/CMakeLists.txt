PROTOBUF_GENERATE_CPP(draco-pnc-msg-src draco-pnc-msg-header draco.proto)
add_library(draco-pnc-msg ${draco-pnc-msg-src} ${draco-pnc-msg-header})
target_include_directories(draco-pnc-msg PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(draco-pnc-msg-py draco.proto)
add_custom_target(draco-pnc-msg-py-target ALL DEPENDS ${draco-pnc-msg-py})


install(TARGETS draco-pnc-msg DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${PROJECT_SOURCE_DIR}/build/messages/draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages")

