# Trajectory Planning Helpers C++

A C++ port of the Python trajectory_planning_helpers library from TUM/FTM, providing trajectory and path planning utilities for autonomous vehicles and racing applications.

## Features

- **Spline Calculations**: Cubic spline interpolation and calculation
- **Curvature Analysis**: Numerical heading and curvature calculation  
- **Velocity Profiling**: Velocity profile generation considering vehicle dynamics
- **Path Optimization**: Minimum curvature path optimization (simplified)
- **Path Matching**: Global and local path matching utilities
- **Utility Functions**: Angle normalization, normal/tangent vector calculation

## Dependencies

- **Eigen3**: Linear algebra library (required)
- **CMake**: Build system (version 3.12+)
- **C++17**: Modern C++ standard

## Installation

1. Install dependencies:
   ```bash
   # Ubuntu/Debian
   sudo apt install libeigen3-dev cmake
   
   # macOS
   brew install eigen cmake
   ```

2. Clone and build:
   ```bash
   git clone <repository>
   cd trajectory_planning_helpers_cpp
   mkdir build && cd build
   cmake ..
   make -j4
   ```

## Usage

### Basic Example

```cpp
#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
using namespace trajectory_planning_helpers;

// Create path points
Matrix2Xd path(2, 4);
path << 0.0, 10.0, 20.0, 30.0,
        0.0, 5.0,  10.0, 15.0;

// Calculate curvature
VectorXd el_lengths(3);
// ... calculate element lengths ...
auto [psi, kappa] = calc_head_curv_num(path, el_lengths, false);

// Calculate velocity profile  
auto [vx_profile, ax_profile] = calc_vel_profile(
    kappa, el_lengths, false, 0.3, 1200.0, VectorXd(), 1.0, 10.0, 5.0);
```

### Spline Example

```cpp
// Create splines from waypoints
double psi_s = M_PI/4;  // Start heading
double psi_e = -M_PI/6; // End heading
auto [coeffs_x, coeffs_y, M, normvec] = calc_splines(waypoints, VectorXd(), psi_s, psi_e);

// Interpolate splines
auto [path_interp, spline_inds, t_values, s_values] = interp_splines(coeffs_x, coeffs_y, 1, 1.0);
```

## API Reference

### Core Functions

- `calc_splines()`: Calculate cubic spline coefficients
- `interp_splines()`: Interpolate points along splines  
- `calc_head_curv_num()`: Calculate heading and curvature numerically
- `calc_vel_profile()`: Generate velocity profile
- `opt_min_curv()`: Optimize for minimum curvature path

### Utility Functions

- `normalize_psi()`: Normalize angle to [-π, π]
- `calc_normal_vectors()`: Calculate normal vectors from heading
- `calc_tangent_vectors()`: Calculate tangent vectors from heading
- `angle3pt()`: Calculate angle between three points

## Examples

Run the provided examples:

```bash
cd build
./examples/basic_example
./examples/spline_example  
```

## Differences from Python Version

This C++ port provides core functionality with some simplifications:

- Simplified velocity profile calculation (no complex GGV diagrams)
- Placeholder optimization (no quadprog dependency) 
- Reduced filter and interpolation options
- Focus on performance and memory efficiency

## License

Based on the original trajectory_planning_helpers (LGPL v3).

## Contributing

Contributions welcome! Please ensure:
- Code follows modern C++ practices
- Functions are tested
- Documentation is updated