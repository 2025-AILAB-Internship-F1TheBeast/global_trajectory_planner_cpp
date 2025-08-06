#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace trajectory_planning_helpers;

int main() {
    std::cout << "Trajectory Planning Helpers C++ - Spline Example" << std::endl;
    
    // Create waypoints for spline fitting
    Matrix2Xd waypoints(2, 4);
    waypoints << 0.0, 10.0, 20.0, 25.0,
                 0.0, 8.0,  10.0, 5.0;
    
    std::cout << "Waypoints:" << std::endl;
    std::cout << waypoints << std::endl;
    
    // Calculate splines
    double psi_s = M_PI/4;  // Start heading: 45 degrees
    double psi_e = -M_PI/6; // End heading: -30 degrees
    
    auto [coeffs_x, coeffs_y, M, normvec] = calc_splines(waypoints, VectorXd(), psi_s, psi_e, true);
    
    std::cout << "\nSpline coefficients X:" << std::endl;
    std::cout << coeffs_x << std::endl;
    
    std::cout << "\nSpline coefficients Y:" << std::endl;
    std::cout << coeffs_y << std::endl;
    
    std::cout << "\nNormal vectors:" << std::endl;
    std::cout << normvec << std::endl;
    
    // Interpolate splines
    auto [path_interp, spline_inds, t_values, s_values] = interp_splines(coeffs_x, coeffs_y, 1, 1.0);
    
    std::cout << "\nInterpolated path (first 10 points):" << std::endl;
    int n_show = std::min(10, int(path_interp.cols()));
    for (int i = 0; i < n_show; ++i) {
        std::cout << "Point " << i << ": (" << path_interp(0,i) << ", " << path_interp(1,i) << ")" << std::endl;
    }
    
    std::cout << "\nArc length values (first 10 points):" << std::endl;
    for (int i = 0; i < n_show; ++i) {
        std::cout << "s[" << i << "] = " << s_values(i) << std::endl;
    }
    
    return 0;
}