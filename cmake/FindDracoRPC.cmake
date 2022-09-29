# - Try to find DracoRPC
#  Once done this will define
#  DRACO_RPC_FOUND - System found RPC
#  DRACO_RPC_INCLUDE_DIRS - DracoRPC include directories
#  DRACO_RPC_LIBRARIES - The libraries needed to use Gurobi

find_path(DRACO_RPC_INCLUDE_DIR
          NAMES configuration.hpp
          PATHS "/usr/local/include/rpc"
          )

find_library(DRACO_RPC_LIBRARY
             NAMES draco-controller
             PATHS "/usr/local/lib"
             )

include(FindPackageHandleStandardArgs)

if(DRACO_RPC_INCLUDE_DIR)
    set(DRACO_RPC_INCLUDE_DIRS "${DRACO_RPC_INCLUDE_DIR}" )
    set(DRACO_RPC_LIBRARIES "${DRACO_RPC_LIBRARY}" )
    set(DRACO_RPC_FOUND TRUE)
    message("-- Found DracoRPC: TRUE")
else()
    message("-- Found DracoRPC: FALSE, Build without DracoRPC")
endif()

mark_as_advanced( DRACO_RPC_INCLUDE_DIR
    DRACO_RPC_LIBRARY )
