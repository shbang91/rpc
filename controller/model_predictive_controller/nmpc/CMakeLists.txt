find_package(matlogger2 REQUIRED)
find_package(muvt_core REQUIRED)

include_directories(${muvt_core_INCLUDE_DIRS})

add_library(nmpc-handler SHARED nmpc_handler.cpp nmpc_handler.hpp)
target_link_libraries(nmpc-handler pin-robot-system
                                   ${Protobuf_LIBRARIES}
                                   ${ZMQ_LIBRARIES}
                                   pnc-to-horizon
                                   horizon-to-pnc
                                   dcm_planner
                                   rpc-util
                                   ${muvt_core_LIBRARIES}
                                   matlogger2::matlogger2)

install(TARGETS nmpc-handler DESTINATION "${INSTALL_LIB_DIR}")
install(FILES nmpc_handler.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/model_predictive_controller/nmpc")
