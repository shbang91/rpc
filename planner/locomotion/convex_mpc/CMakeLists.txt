project(convex-mpc CXX)

######################
## Build convex-mpc ##
######################
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS include/convex_mpc/*.h*)

#include(CMakePrintHelpers)
#cmake_print_variables(${PROJECT_NAME}_HEADERS)
#cmake_print_variables(${PROJECT_NAME}_SOURCES)

add_library(
  ${PROJECT_NAME} 
  SHARED
  ${${PROJECT_NAME}_SOURCES} 
  ${${PROJECT_NAME}_HEADERS}
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC
  #${PINOCCHIO_LIBRARIES}
  #pinocchio::pinocchio
  hpipm-cpp::hpipm-cpp
  rpc-util
  rpc-pin-robot-system
  rpc-wbc
)

target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  #${PINOCCHIO_INCLUDE_DIRS}
  #${pinocchio_INCLUDE_DIRS}
  ${hpipm-cpp_INCLUDE_DIRS}
  #$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
#if (OPTIMIZE_FOR_NATIVE)
  #target_compile_options(
    #${PROJECT_NAME} 
    #PUBLIC
    #-march=native
  #)
#endif()

#############
## Testing ##
#############
# Add testing directory
if (BUILD_WITH_GTESTS)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif() # end if (BUILD_TESTS)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${${PROJECT_NAME}_HEADERS} DESTINATION
    "${INSTALL_INCLUDE_DIR}/planner/locomotion/convex_mpc/include/convex_mpc")
