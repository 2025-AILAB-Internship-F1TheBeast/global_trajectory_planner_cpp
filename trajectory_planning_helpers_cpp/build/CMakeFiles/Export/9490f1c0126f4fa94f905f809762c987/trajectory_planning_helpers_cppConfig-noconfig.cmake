#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "trajectory_planning_helpers" for configuration ""
set_property(TARGET trajectory_planning_helpers APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(trajectory_planning_helpers PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libtrajectory_planning_helpers.a"
  )

list(APPEND _cmake_import_check_targets trajectory_planning_helpers )
list(APPEND _cmake_import_check_files_for_trajectory_planning_helpers "${_IMPORT_PREFIX}/lib/libtrajectory_planning_helpers.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
