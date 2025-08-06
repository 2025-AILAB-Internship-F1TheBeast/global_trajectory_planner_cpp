cmake_minimum_required(VERSION 3.12)
project(trajectory_planning_helpers_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Library sources
set(SOURCES
    src/calc_splines.cpp
    src/calc_head_curv_num.cpp
    src/calc_normal_vectors.cpp
    src/calc_tangent_vectors.cpp
    src/interp_splines.cpp
    src/opt_min_curv.cpp
    src/calc_vel_profile.cpp
    src/spline_approximation.cpp
    src/normalize_psi.cpp
    src/angle3pt.cpp
    src/path_matching_global.cpp
    src/path_matching_local.cpp
)

# Create library
add_library(trajectory_planning_helpers ${SOURCES})

# Link libraries
target_link_libraries(trajectory_planning_helpers 
    Eigen3::Eigen
)

# Set include directories for the library
target_include_directories(trajectory_planning_helpers
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Examples
add_subdirectory(examples)

# Tests (optional)
option(BUILD_TESTS "Build test programs" ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Install configuration
install(TARGETS trajectory_planning_helpers
    EXPORT trajectory_planning_helpers_cppConfig
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT trajectory_planning_helpers_cppConfig
    FILE trajectory_planning_helpers_cppConfig.cmake
    DESTINATION lib/cmake/trajectory_planning_helpers_cpp
)