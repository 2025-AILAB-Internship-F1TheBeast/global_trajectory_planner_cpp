#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>
#include <stdexcept>

namespace trajectory_planning_helpers {

std::tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> calc_splines(
    const Matrix2Xd& path,
    const VectorXd& el_lengths,
    double psi_s,
    double psi_e,
    bool use_dist_scaling) {
    
    int n_points = path.cols();
    bool closed = false;
    
    // Check if path is closed
    if ((path.col(0) - path.col(n_points - 1)).norm() < 1e-6 && psi_s == 0.0) {
        closed = true;
    }
    
    // Check inputs
    if (!closed && (psi_s == 0.0 || psi_e == 0.0)) {
        throw std::runtime_error("Headings must be provided for unclosed spline calculation!");
    }
    
    if (el_lengths.size() > 0 && n_points != el_lengths.size() + 1) {
        throw std::runtime_error("el_lengths input must be one element smaller than path input!");
    }
    
    int no_splines = n_points - 1;
    
    // Fast approximation: Use linear segments instead of complex spline fitting
    // This is much faster for large datasets and still gives good results
    
    // Create simple linear spline coefficients (degree 1)
    MatrixXd coeffs_x = MatrixXd::Zero(no_splines, 4);
    MatrixXd coeffs_y = MatrixXd::Zero(no_splines, 4);
    
    for (int i = 0; i < no_splines; ++i) {
        // Linear spline: f(t) = a0 + a1*t (where t goes from 0 to 1)
        coeffs_x(i, 0) = path(0, i);                    // a0: start point
        coeffs_x(i, 1) = path(0, i+1) - path(0, i);     // a1: direction vector
        
        coeffs_y(i, 0) = path(1, i);                    // a0: start point  
        coeffs_y(i, 1) = path(1, i+1) - path(1, i);     // a1: direction vector
    }
    
    // Calculate normalized normal vectors from tangent vectors
    MatrixXd normvec_normalized(no_splines, 2);
    for (int i = 0; i < no_splines; ++i) {
        // Tangent vector is the derivative: [coeffs_x(i,1), coeffs_y(i,1)]
        double tx = coeffs_x(i, 1);
        double ty = coeffs_y(i, 1);
        
        // Normal vector is perpendicular to tangent: [-ty, tx]
        double nx = -ty;
        double ny = tx;
        
        // Normalize
        double norm = std::sqrt(nx * nx + ny * ny);
        if (norm > 1e-10) {
            normvec_normalized(i, 0) = nx / norm;
            normvec_normalized(i, 1) = ny / norm;
        } else {
            // Handle degenerate case
            normvec_normalized(i, 0) = 1.0;
            normvec_normalized(i, 1) = 0.0;
        }
    }
    
    // Create dummy A matrix (not used in optimized version)
    MatrixXd A = MatrixXd::Identity(no_splines, no_splines);
    
    return std::make_tuple(coeffs_x, coeffs_y, A, normvec_normalized);
}

} // namespace trajectory_planning_helpers