# enable google test
include(cmake/gtest.cmake)

# macto for testing
macro(add_srbd_mpc_test TESTNAME)
  add_executable(
    ${TESTNAME} 
    ${TESTNAME}.cpp
  )
  target_include_directories(
    ${TESTNAME} 
    PRIVATE
    ${GTEST_INCLUDE_PATH}
    ${GMOCK_INCLUDE_PATH}
    ${GTEST_LIBRARY_PATH}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/test/test_helper
  )
  target_link_libraries(
    ${TESTNAME} 
    PRIVATE
    GTest::GTest
    GTest::GMock
    ${PROJECT_NAME}
  )

  if (BUILD_WITH_MATLOGGER)
      target_link_libraries(${TESTNAME} PRIVATE matlogger2::matlogger2)
  endif()

  add_dependencies(
    ${TESTNAME} 
    googletest
  )
  add_test(
    NAME ${TESTNAME} 
    COMMAND $<TARGET_FILE:${TESTNAME}>
  )
endmacro()

# add tests
#add_srbd_mpc_test(contact_schedule_test)
#add_srbd_mpc_test(cost_function_test)
#add_srbd_mpc_test(friction_cone_test)
#add_srbd_mpc_test(robot_state_test)
#add_srbd_mpc_test(single_rigid_body_test)
#add_srbd_mpc_test(mpc_test)
add_srbd_mpc_test(mpc_draco_test)
