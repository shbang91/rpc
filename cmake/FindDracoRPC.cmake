# - Try to find DracoRPC
#  Once done this will define
#  DRACO_RPC_FOUND - System found RPC
#  DRACO_RPC_INCLUDE_DIRS - DracoRPC include directories
#  DRACO_RPC_LIBRARIES -

include(FindPackageHandleStandardArgs)

find_path(DRACO_RPC_INCLUDE_DIR
          NAMES configuration.hpp
          PATHS "/usr/local/include/rpc"
          )
find_library(DRACO_RPC_LIBRARY
             NAMES rpc-draco-controller
             PATHS "/usr/local/lib"
             )
#find_library(ROBOT_SYSTEM_LIBRARY
             #NAMES rpc-pin-robot-system
             #PATHS "/usr/local/lib"
             #)
#find_library(DRACO_MSG
             #NAMES rpc-draco-msg
             #PATHS "/usr/local/lib"
             #)
#find_library(DRACO_FILTER
             #NAMES rpc-filter
             #PATHS "/usr/local/lib"
             #)
#find_library(DRACO_SOLVER
             #NAMES rpc-goldfarb
             #PATHS "/usr/local/lib"
             #)
#find_library(PINOCCHIO
    #NAMES rpc-pin-robot-system
    #PATHS "/usr/local/lib"
    #)
#find_library(UTIL
    #NAMES rpc-util
    #PATHS "/usr/local/lib"
    #)
#find_library(WBC
    #NAMES rpc-wbc
    #PATHS "/usr/local/lib"
    #)
#find_library(YAML
    #NAMES rpc-yaml
    #PATHS "/usr/local/lib"
    #)
#find_library(LMPC_HANDLER
    #NAMES rpc-lmpc-handler
    #PATHS "/usr/local/lib"
    #)
#find_library(DCM_PLANNER
    #NAMES rpc-dcm-planner
    #PATHS "/usr/local/lib"
    #)

if(DRACO_RPC_INCLUDE_DIR)
    set(DRACO_RPC_INCLUDE_DIRS "${DRACO_RPC_INCLUDE_DIR}" )
    set(DRACO_RPC_LIBRARIES "${DRACO_RPC_LIBRARY}" )
    #set(DRACO_RPC_ROBOT_SYSTEM "${ROBOT_SYSTEM_LIBRARY}" )
    set(DRACO_RPC_FOUND TRUE)
    message("-- Found DracoRPC: TRUE")
else()
    message("-- Found DracoRPC: FALSE, Build without DracoRPC")
endif()

mark_as_advanced(DRACO_RPC_INCLUDE_DIR DRACO_RPC_LIBRARY )
