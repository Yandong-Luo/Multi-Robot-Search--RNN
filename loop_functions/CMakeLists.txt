include_directories(${CMAKE_SOURCE_DIR})

# Descend into the foraging_loop_functions directory
#add_subdirectory(foraging_loop_functions)

# If Qt+OpenGL dependencies were found, descend into these directories
#if(ARGOS_COMPILE_QTOPENGL)
#  add_subdirectory(trajectory_loop_functions)
#  add_subdirectory(single_species_functions)
#  add_subdirectory(single_species_no_floor_functions)
  # add_subdirectory(RNN_loop_functions)
#endif(ARGOS_COMPILE_QTOPENGL)

# If GALIB was found, also descend into galib_phototaxis_loop_functions
if(GALIB_FOUND)
  add_subdirectory(RNN_loop_functions)
endif(GALIB_FOUND)
