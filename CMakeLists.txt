cmake_minimum_required(VERSION 3.5.1)

project(rpc)

##==================================================
## CMake Flags
##==================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS on)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


##==================================================
## Build Setup
##==================================================
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
set(PYTHON_BINDING TRUE)

##==================================================
## Include Directory
##==================================================
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/build")
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

##==================================================
## Dependency
##==================================================

# DART
find_package(DART 6.10 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

# Eigen
find_package(Eigen 3.3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
##==================================================
## Subdirectories
##==================================================
add_subdirectory(pnc)
add_subdirectory(test)
#add_subdirectory(util)


##==================================================
## Configuration File
##==================================================
set(THIS_COM "${PROJECT_SOURCE_DIR}/")
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake
    ${PROJECT_SOURCE_DIR}/configuration.hpp)
install(FILES ${PROJECT_SOURCE_DIR}/configuration.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")

