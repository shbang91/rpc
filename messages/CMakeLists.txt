PROTOBUF_GENERATE_CPP(draco-pnc-msg-src draco-pnc-msg-header draco.proto)
add_library(draco-pnc-msg ${draco-pnc-msg-src} ${draco-pnc-msg-header})
target_include_directories(draco-pnc-msg PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(draco-pnc-msg-py draco.proto)
add_custom_target(draco-pnc-msg-py-target ALL DEPENDS ${draco-pnc-msg-py})

PROTOBUF_GENERATE_CPP(fixed-draco-pnc-msg-src fixed-draco-pnc-msg-header
    fixed_draco.proto)
add_library(fixed-draco-pnc-msg ${fixed-draco-pnc-msg-src} ${fixed-draco-pnc-msg-header})
target_include_directories(fixed-draco-pnc-msg PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(fixed-draco-pnc-msg-py fixed_draco.proto)
add_custom_target(fixed-draco-pnc-msg-py-target ALL DEPENDS ${fixed-draco-pnc-msg-py})


install(TARGETS draco-pnc-msg DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${PROJECT_SOURCE_DIR}/build/messages/draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages")

install(TARGETS fixed-draco-pnc-msg DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${PROJECT_SOURCE_DIR}/build/messages/fixed_draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages")
