file(GLOB_RECURSE sources "*.cpp" "*.c")
file(GLOB controller_headers "*.hpp")
file(GLOB state_machines_headers "draco_state_machines/*.hpp")
file(GLOB state_machines_headers_wbic "draco_state_machines_wbic/*.hpp")
file(GLOB draco_task_headers "draco_task/*.hpp" "draco_task/*.h")
file(GLOB draco_crbi_headers "draco_crbi/*.hpp" "draco_crbi/*.h")

list(REMOVE_ITEM sources "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.cpp")
list(REMOVE_ITEM controller_headers "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.hpp")

list(REMOVE_ITEM sources "${CMAKE_CURRENT_SOURCE_DIR}/draco_rs_teleop_handler.cpp")
list(REMOVE_ITEM controller_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/draco_rs_teleop_handler.hpp")
list(REMOVE_ITEM sources "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines/teleop_manipulation.cpp")
list(REMOVE_ITEM state_machines_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines/teleop_manipulation.hpp")

if(BUILD_WITH_ZMQ_PROTOBUF)
    list(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.cpp")
    list(APPEND controller_headers "${CMAKE_CURRENT_SOURCE_DIR}/draco_data_manager.hpp")
endif()

if(BUILD_WITH_TELEOP)
    list(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/draco_rs_teleop_handler.cpp")
    list(APPEND controller_headers "${CMAKE_CURRENT_SOURCE_DIR}/draco_rs_teleop_handler.hpp")
    list(APPEND sources
        "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines/teleop_manipulation.cpp")
    list(APPEND state_machines_headers "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines/teleop_manipulation.hpp")
endif()

list(REMOVE_ITEM sources
    "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines_wbic/mpc_locomotion.cpp")
list(REMOVE_ITEM state_machines_headers_wbic
    "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines_wbic/mpc_locomotion.hpp")

if(BUILD_WITH_HPIPM)
    list(APPEND sources
    "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines_wbic/mpc_locomotion.cpp")
list(APPEND state_machines_headers_wbic
    "${CMAKE_CURRENT_SOURCE_DIR}/draco_state_machines_wbic/mpc_locomotion.hpp")
endif()

#list(REMOVE_ITEM sources
    #"${CMAKE_CURRENT_SOURCE_DIR}/draco_task/draco_wbo_task.cpp")
#list(REMOVE_ITEM controller_headers
    #"${CMAKE_CURRENT_SOURCE_DIR}/draco_task/draco_wbo_task.hpp")

#file(GLOB wbo_source
    #"${CMAKE_CURRENT_SOURCE_DIR}/draco_task/draco_wbo_task.cpp")
#file(GLOB wbo_header
    #"${CMAKE_CURRENT_SOURCE_DIR}/draco_task/draco_wbo_task.hpp")

#include(CMakePrintHelpers)
#cmake_print_variables(sources)
#cmake_print_variables(controller_headers)

add_library(rpc-draco-controller SHARED ${sources} ${controller_headers}
    ${state_machines_headers} ${state_machines_headers_wbic} ${draco_task_headers} ${draco_crbi_headers})


target_link_libraries(rpc-draco-controller PUBLIC rpc-pin-robot-system
                                           PUBLIC rpc-wbc
                                           PUBLIC rpc-util
                                           PUBLIC rpc-dcm-planner
                                           PUBLIC rpc-filter
                                           PUBLIC rpc-state-estimator
                                           PUBLIC rpc-model
                                           )

if(BUILD_WITH_ZMQ_PROTOBUF)
    target_link_libraries(rpc-draco-controller PUBLIC ${Protobuf_LIBRARIES}
                                               PUBLIC ${ZMQ_LIBRARIES}
                                               PUBLIC rpc-draco-msg)
endif()

if(BUILD_WITH_HPIPM)
    target_link_libraries(rpc-draco-controller PUBLIC convex-mpc)
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
install(FILES ${state_machines_headers_wbic} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_state_machines_wbic")
install(FILES ${draco_task_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_task")
install(FILES ${draco_crbi_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_crbi")
