#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "global_racetrajectory_optimization" for configuration ""
set_property(TARGET global_racetrajectory_optimization APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(global_racetrajectory_optimization PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libglobal_racetrajectory_optimization.a"
  )

list(APPEND _cmake_import_check_targets global_racetrajectory_optimization )
list(APPEND _cmake_import_check_files_for_global_racetrajectory_optimization "${_IMPORT_PREFIX}/lib/libglobal_racetrajectory_optimization.a" )

# Import target "global_trajectory_optimizer" for configuration ""
set_property(TARGET global_trajectory_optimizer APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(global_trajectory_optimizer PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/global_trajectory_optimizer"
  )

list(APPEND _cmake_import_check_targets global_trajectory_optimizer )
list(APPEND _cmake_import_check_files_for_global_trajectory_optimizer "${_IMPORT_PREFIX}/bin/global_trajectory_optimizer" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
