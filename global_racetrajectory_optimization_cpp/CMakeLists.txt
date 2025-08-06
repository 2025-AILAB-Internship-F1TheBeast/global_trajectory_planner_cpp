cmake_minimum_required(VERSION 3.12)
project(global_racetrajectory_optimization_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)

# Find trajectory_planning_helpers_cpp (assume it's built and installed)
# If not installed system-wide, adjust path accordingly
find_path(TPH_INCLUDE_DIR NAMES trajectory_planning_helpers/trajectory_planning_helpers.hpp
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planning_helpers_cpp/include)

find_library(TPH_LIBRARY NAMES trajectory_planning_helpers
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planning_helpers_cpp/build)

if(NOT TPH_INCLUDE_DIR OR NOT TPH_LIBRARY)
    message(STATUS "trajectory_planning_helpers_cpp not found, building from source...")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planning_helpers_cpp trajectory_planning_helpers_cpp)
    set(TPH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planning_helpers_cpp/include)
    set(TPH_LIBRARY trajectory_planning_helpers)
endif()

# Optional: Find optimization solver (OSQP)
find_package(osqp QUIET)

# Include directories
include_directories(include ${TPH_INCLUDE_DIR})

# Library sources
set(SOURCES
    src/global_race_trajectory_optimization.cpp
    src/track_preparation.cpp
    src/optimization_interface.cpp
    src/vehicle_parameters.cpp
    src/result_export.cpp
    src/config_parser.cpp
)

# Create library
add_library(global_racetrajectory_optimization ${SOURCES})

# Link libraries
target_link_libraries(global_racetrajectory_optimization 
    ${TPH_LIBRARY}
    Eigen3::Eigen
)

if(osqp_FOUND)
    target_link_libraries(global_racetrajectory_optimization osqp::osqp)
    target_compile_definitions(global_racetrajectory_optimization PRIVATE HAS_OSQP)
endif()

# Set include directories for the library
target_include_directories(global_racetrajectory_optimization
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Create main executable
add_executable(global_trajectory_optimizer 
    src/main_global_trajectory.cpp
)

target_link_libraries(global_trajectory_optimizer 
    global_racetrajectory_optimization
)

# Examples
add_subdirectory(examples)

# Tests (optional)
option(BUILD_TESTS "Build test programs" ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Install configuration
install(TARGETS global_racetrajectory_optimization global_trajectory_optimizer
    EXPORT global_racetrajectory_optimization_cppConfig
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY inputs/ DESTINATION share/global_racetrajectory_optimization_cpp/inputs)
install(DIRECTORY params/ DESTINATION share/global_racetrajectory_optimization_cpp/params)

install(EXPORT global_racetrajectory_optimization_cppConfig
    FILE global_racetrajectory_optimization_cppConfig.cmake
    DESTINATION lib/cmake/global_racetrajectory_optimization_cpp
)