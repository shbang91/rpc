add_library(rpc-lmpc_handler SHARED lmpc_handler.cpp lmpc_handler.hpp)
target_link_libraries(rpc-lmpc_handler rpc-dcm_planner
                                    rpc-pin-robot-system
                                    rpc-util)

install(TARGETS rpc-lmpc_handler DESTINATION "${INSTALL_LIB_DIR}")
install(FILES lmpc_handler.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/model_predictive_controller/lmpc")


