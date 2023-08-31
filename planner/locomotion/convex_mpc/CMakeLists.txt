project(convex-mpc CXX)


######################
## Build convex-mpc ##
######################
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS include/${PROJECT_NAME}/*.h*)
add_library(
  ${PROJECT_NAME} 
  SHARED
  ${${PROJECT_NAME}_SOURCES} 
  ${${PROJECT_NAME}_HEADERS}
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC
  ${PINOCCHIO_LIBRARIES}
  hpipm-cpp::hpipm-cpp
  rpc-util
)
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${hpipm-cpp_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if (OPTIMIZE_FOR_NATIVE)
  target_compile_options(
    ${PROJECT_NAME} 
    PUBLIC
    -march=native
  )
endif()

#############
## Testing ##
#############
# Add testing directory
if (BUILD_TESTING)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif() # end if (BUILD_TESTS)

#############
## Install ##
#############
#TODO
