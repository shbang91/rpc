file(GLOB_RECURSE sources "*.cpp")
file(GLOB controller_headers "*.hpp")
file(GLOB state_machines_headers "draco_state_machines/*.hpp")
file(GLOB draco_task_headers "draco_task/*.hpp")

list(REMOVE_ITEM sources "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.cpp")
list(REMOVE_ITEM controller_headers "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.hpp")


if(BUILD_WITH_ZMQ_PROTOBUF)
    list(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.cpp")
    list(APPEND controller_headers "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.hpp")
endif()

#include(CMakePrintHelpers)
#cmake_print_variables(sources)
#cmake_print_variables(controller_headers)

add_library(rpc-draco-controller SHARED ${sources} ${controller_headers}
    ${state_machines_headers} ${draco_task_headers})

target_link_libraries(rpc-draco-controller PUBLIC rpc-pin-robot-system
                                           PUBLIC rpc-wbc
                                           PUBLIC rpc-util
                                           PUBLIC rpc-dcm-planner
                                           PUBLIC rpc-lmpc-handler
                                           PUBLIC rpc-filter
                                           PUBLIC rpc-state-estimator)

if(BUILD_WITH_ZMQ_PROTOBUF)
    target_link_libraries(rpc-draco-controller PUBLIC ${Protobuf_LIBRARIES}
                                               PUBLIC ${ZMQ_LIBRARIES}
                                               PUBLIC rpc-draco-msg)
endif()

if(BUILD_WITH_MATLOGGER)
    target_link_libraries(rpc-draco-controller PUBLIC matlogger2::matlogger2)
endif()

if(BUILD_WITH_FOXGLOVE)
    target_link_libraries(rpc-draco-controller PUBLIC foxglove-parameter-subscriber)
endif()

install(TARGETS rpc-draco-controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${controller_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller")
install(FILES ${state_machines_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_state_machines")
install(FILES ${draco_task_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_task")


