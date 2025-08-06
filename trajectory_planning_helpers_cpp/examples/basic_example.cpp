#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <iostream>
#include <iomanip>

using namespace trajectory_planning_helpers;

int main() {
    std::cout << "Trajectory Planning Helpers C++ - Basic Example" << std::endl;
    
    // Create a simple test path
    Matrix2Xd path(2, 4);
    path << 0.0, 10.0, 20.0, 30.0,
            0.0, 5.0,  10.0, 15.0;
    
    std::cout << "Input path:" << std::endl;
    std::cout << path << std::endl;
    
    // Calculate element lengths
    VectorXd el_lengths(3);
    for (int i = 0; i < 3; ++i) {
        el_lengths(i) = (path.col(i+1) - path.col(i)).norm();
    }
    
    std::cout << "\nElement lengths:" << std::endl;
    std::cout << el_lengths.transpose() << std::endl;
    
    // Calculate heading and curvature
    auto [psi, kappa] = calc_head_curv_num(path, el_lengths, false, 1.0, 1.0, 2.0, 2.0, true);
    
    std::cout << "\nHeading (psi):" << std::endl;
    std::cout << psi.transpose() << std::endl;
    
    std::cout << "\nCurvature (kappa):" << std::endl;  
    std::cout << kappa.transpose() << std::endl;
    
    // Calculate velocity profile
    auto [vx_profile, ax_profile] = calc_vel_profile(kappa, el_lengths, false, 0.3, 1200.0, VectorXd(), 1.0, 10.0, 5.0);
    
    std::cout << "\nVelocity profile:" << std::endl;
    std::cout << vx_profile.transpose() << std::endl;
    
    std::cout << "\nAcceleration profile:" << std::endl;
    std::cout << ax_profile.transpose() << std::endl;
    
    return 0;
}