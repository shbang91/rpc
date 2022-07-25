PROTOBUF_GENERATE_CPP(draco-rpc-msg-src draco-rpc-msg-header draco.proto)
add_library(draco-rpc-msg ${draco-rpc-msg-src} ${draco-rpc-msg-header})
target_include_directories(draco-rpc-msg PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(draco-rpc-msg-py draco.proto)
add_custom_target(draco-rpc-msg-py-target ALL DEPENDS ${draco-rpc-msg-py})

PROTOBUF_GENERATE_CPP(pnc-to-horizon-src pnc-to-horizon-header pnc_to_horizon.proto)
add_library(pnc-to-horizon ${pnc-to-horizon-src} ${pnc-to-horizon-header})
target_include_directories(pnc-to-horizon PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(pnc-to-horizon-py pnc_to_horizon.proto)
add_custom_target(pnc-to-horizon-py-target ALL DEPENDS ${pnc-to-horizon-py})

PROTOBUF_GENERATE_CPP(horizon-to-pnc-src horizon-to-pnc-header horizon_to_pnc.proto)
add_library(horizon-to-pnc ${horizon-to-pnc-src} ${horizon-to-pnc-header})
target_include_directories(horizon-to-pnc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(horizon-to-pnc-py horizon_to_pnc.proto)
add_custom_target(horizon-to-pnc-py-target ALL DEPENDS ${horizon-to-pnc-py})

install(TARGETS pnc-to-horizon DESTINATION "${INSTALL_LIB_DIR}")
install(TARGETS horizon-to-pnc DESTINATION "${INSTALL_LIB_DIR}")
install(TARGETS draco-rpc-msg DESTINATION "${INSTALL_LIB_DIR}")

install(FILES ${PROJECT_SOURCE_DIR}/build/messages/draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages")
