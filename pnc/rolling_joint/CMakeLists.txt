file(GLOB rj_sources "*.cpp")
file(GLOB rj_headers "*.hpp")

add_library(rj-module ${rj_sources})
target_link_libraries(rj-module ${DART_LIBRARIES}
                                ${Boost_LIBRARIES})

install(TARGETS rj-module DESTINATION "${INSTALL_LIB_DIR}")
#install(TARGETS ${rj_headers} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/pnc/rolling_joint")
