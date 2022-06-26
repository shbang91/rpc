file(GLOB dart_robot_system_source "dart_robot_system.cpp")
file(GLOB dart_robot_system_header "dart_robot_system.hpp")
#file(GLOB pinocchio_robot_system_source "pinocchio_robot_system.cpp")
#file(GLOB pinocchio_robot_system_header "pinocchio_robot_system.hpp")

add_library(dart-robot-system SHARED ${dart_robot_system_source}
    ${dart_robot_system_header})

target_link_libraries(dart-robot-system ${DART_LIBRARIES}
                                    rpc-util)

#add_library(pinocchio-robot-system SHARED ${pinocchio_robot_system_source}
    #${pinocchio_robot_system_header})

#target_link_libraries(pinocchio-robot-system ${PINOCCHIO_LIBRARIES}
                                    #pnc-util)

install(TARGETS dart-robot-system DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${dart_robot_system_header} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/robot_system")
#install(TARGETS pinocchio-robot-system DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${pinocchio_robot_system_header} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/pnc/robot_system")
