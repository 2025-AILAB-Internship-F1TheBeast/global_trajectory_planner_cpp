#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>
#include <limits>

namespace trajectory_planning_helpers {

std::tuple<int, double> path_matching_local(
    const Vector2d& pos_est,
    const Matrix2Xd& reftrack,
    int s_ind_last_guess,
    const VectorXd& el_lengths) {
    
    int n_reftrack_points = reftrack.cols();
    
    // Search window around last guess
    int search_window = 10;
    int start_idx = std::max(0, s_ind_last_guess - search_window);
    int end_idx = std::min(n_reftrack_points - 1, s_ind_last_guess + search_window);
    
    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = s_ind_last_guess;
    
    for (int i = start_idx; i <= end_idx; ++i) {
        double dist = (pos_est - reftrack.col(i)).norm();
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    // Calculate interpolation parameter t
    double t = 0.0;
    
    if (closest_idx < n_reftrack_points - 1) {
        Vector2d p1 = reftrack.col(closest_idx);
        Vector2d p2 = reftrack.col(closest_idx + 1);
        Vector2d segment = p2 - p1;
        Vector2d to_point = pos_est - p1;
        
        double segment_length_sq = segment.squaredNorm();
        if (segment_length_sq > 1e-10) {
            t = std::max(0.0, std::min(1.0, to_point.dot(segment) / segment_length_sq));
        }
    }
    
    return std::make_tuple(closest_idx, t);
}

} // namespace trajectory_planning_helpers