
file(GLOB solvers "*.c")



foreach(solver_path ${solvers})
    get_filename_component(solver ${solver_path} NAME_WE)  # Extracting solver name without extension
    message("solver :   "   ${solver} "  " ${solver_path})
    add_library(${solver} SHARED ${solver_path})
    set_target_properties(${solver} PROPERTIES IMPORTED_LOCATION ${solver_path} )
    set_target_properties(${solver} PROPERTIES PREFIX "")
    set_target_properties(${solver} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib")
    install(TARGETS ${solver} DESTINATION "${INSTALL_LIB_DIR}")
endforeach()



