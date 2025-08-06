#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>
#include <iostream>

namespace trajectory_planning_helpers {

std::tuple<MatrixXd, VectorXd> spline_approximation(
    const Matrix2Xd& track,
    int k_reg,
    double s_reg,
    int stepsize_prep,
    double stepsize_reg,
    bool debug) {
    
    // Simplified and fast spline approximation implementation
    // This uses linear interpolation and resampling for performance
    
    int n_points = track.cols();
    
    // Calculate cumulative distances
    VectorXd el_lengths_orig(n_points - 1);
    for (int i = 0; i < n_points - 1; ++i) {
        el_lengths_orig(i) = (track.col(i + 1) - track.col(i)).norm();
    }
    
    // Calculate total track length
    double total_length = el_lengths_orig.sum();
    
    // Determine number of output points based on desired stepsize
    double effective_stepsize = stepsize_reg;  // Use parameter from config
    int n_out = std::max(10, static_cast<int>(std::ceil(total_length / effective_stepsize)));
    
    // Create uniform spacing
    VectorXd s_uniform = VectorXd::LinSpaced(n_out, 0.0, total_length);
    
    // Calculate cumulative distances for original track
    VectorXd s_orig(n_points);
    s_orig(0) = 0.0;
    for (int i = 1; i < n_points; ++i) {
        s_orig(i) = s_orig(i-1) + el_lengths_orig(i-1);
    }
    
    // Interpolate track points to uniform spacing
    MatrixXd track_out(n_out, 2);
    
    for (int i = 0; i < n_out; ++i) {
        double s_target = s_uniform(i);
        
        // Find the segment containing s_target
        int seg_idx = 0;
        for (int j = 1; j < n_points; ++j) {
            if (s_orig(j) > s_target) {
                seg_idx = j - 1;
                break;
            }
            seg_idx = j;
        }
        
        // Handle edge case
        if (seg_idx >= n_points - 1) {
            seg_idx = n_points - 2;
        }
        
        // Linear interpolation within the segment
        double seg_length = el_lengths_orig(seg_idx);
        double local_s = s_target - s_orig(seg_idx);
        double t = (seg_length > 1e-10) ? (local_s / seg_length) : 0.0;
        t = std::max(0.0, std::min(1.0, t)); // Clamp to [0,1]
        
        // Interpolate x and y
        track_out(i, 0) = (1.0 - t) * track(0, seg_idx) + t * track(0, seg_idx + 1);
        track_out(i, 1) = (1.0 - t) * track(1, seg_idx) + t * track(1, seg_idx + 1);
    }
    
    // Calculate element lengths for output track
    VectorXd el_lengths_out(n_out - 1);
    for (int i = 0; i < n_out - 1; ++i) {
        el_lengths_out(i) = (track_out.row(i + 1) - track_out.row(i)).norm();
    }
    
    if (debug) {
        std::cout << "Spline approximation: " << n_points << " -> " << n_out 
                  << " points, length: " << total_length << " m, stepsize_reg: " << stepsize_reg << " m" << std::endl;
    }
    
    return std::make_tuple(track_out, el_lengths_out);
}

} // namespace trajectory_planning_helpers