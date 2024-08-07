cmake_minimum_required(VERSION 3.5.1)

project(rpc)

##==================================================
## CMake Flags
##==================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS on)
add_compile_options(-fPIC)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/build/Release/generators/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

##==================================================
## Build Setup
##==================================================
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})

option(PYTHON_BINDING "use pybind11" ON)

option(BUILD_WITH_PINOCCHIO "compile with PINOCCHIO" ON)
option(BUILD_WITH_DART "compile with DART" OFF)

option(BUILD_WITH_MATLOGGER "compile with MatLogger2" OFF)
option(BUILD_WITH_ZMQ_PROTOBUF "compile with zmq and protobuf" OFF)

option(BUILD_WITH_FOXGLOVE "compile with foxglove" OFF)

option(BUILD_WITH_TELEOP "compile with teleop" OFF)

# unit-testing tools
option(BUILD_GTESTS "build google tests" ON)
option(FETCH_GTEST_PKG "use fetch pkg to clone google test" OFF)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()

##==================================================
## Include Directory
##==================================================
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/build")

##==================================================
## Dependency
##==================================================

#pybind11
if(PYTHON_BINDING)
    find_package(pybind11 REQUIRED)
    include_directories(${pybind11_INCLUDE_DIRS})
    add_subdirectory(binding)
endif()

# Pinocchio
if(BUILD_WITH_PINOCCHIO)
    find_package(pinocchio)
    include_directories(${pinocchio_INCLUDE_DIRS})
endif()

# DART
if(BUILD_WITH_DART)
    find_package(DART 6.10 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
    include_directories(${DART_INCLUDE_DIRS})
endif()

# Eigen
find_package(Eigen 3.3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

## MatLogger2
if(BUILD_WITH_MATLOGGER)
    find_package(matlogger2 REQUIRED)
    set(B_USE_MATLOGGER true)
else()
    set(B_USE_MATLOGGER false)
endif()

#Protobuf & ZMQ
if (BUILD_WITH_ZMQ_PROTOBUF OR BUILD_WITH_TELEOP)
    find_package(ZMQ REQUIRED)
    find_package(Protobuf REQUIRED)
    if(Protobuf_FOUND AND ZMQ_FOUND)
        include_directories(${Protobuf_INCLUDE_DIRS})
        include_directories(${ZMQ_INCLUDE_DIRS})
        add_subdirectory(messages)
    endif()
    set(B_USE_ZMQ true)
else()
    set(B_USE_ZMQ false)
endif()

if (BUILD_WITH_TELEOP)
    set(B_USE_TELEOP true)
else()
    set(B_USE_TELEOP false)
endif()

#Boost
#find_package(Boost QUIET REQUIRED COMPONENTS system thread filesystem)
#include_directories(${Boost_INCLUDE_DIRS})

# Foxglove for UI
if(BUILD_WITH_FOXGLOVE)
    find_package(foxglove-schemas-protobuf REQUIRED)
    find_package(foxglove-websocket REQUIRED)
    find_package(Boost COMPONENTS program_options REQUIRED)

    # warning message if trying to build with foxglove but without zmq/protobuf
    if (NOT BUILD_WITH_ZMQ_PROTOBUF)
        message("\n*** WARNING ***\nPLEASE ENABLE ZMQ/PROTOBUF -- needed dependencies\n")
    endif ()
    set(B_USE_FOXGLOVE true)
else()
    set(B_USE_FOXGLOVE false)
endif()



##==================================================
## Subdirectories
##==================================================
add_subdirectory(controller)
add_subdirectory(planner)
add_subdirectory(test)
add_subdirectory(util)
add_subdirectory(third_party)
add_subdirectory(UI)

##==================================================
## Configuration File
##==================================================
set(THIS_COM "${PROJECT_SOURCE_DIR}/")
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake
    ${PROJECT_SOURCE_DIR}/configuration.hpp)
install(FILES ${PROJECT_SOURCE_DIR}/configuration.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")
