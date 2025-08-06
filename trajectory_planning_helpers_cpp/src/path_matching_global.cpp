#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>
#include <limits>

namespace trajectory_planning_helpers {

VectorXd path_matching_global(
    const Matrix2Xd& path,
    const Matrix2Xd& reftrack,
    const VectorXd& el_lengths_reftrack) {
    
    int n_path_points = path.cols();
    int n_reftrack_points = reftrack.cols();
    
    VectorXd s_interp(n_path_points);
    
    // Calculate cumulative distances along reference track
    VectorXd s_reftrack(n_reftrack_points);
    s_reftrack(0) = 0.0;
    
    for (int i = 1; i < n_reftrack_points; ++i) {
        if (i - 1 < el_lengths_reftrack.size()) {
            s_reftrack(i) = s_reftrack(i - 1) + el_lengths_reftrack(i - 1);
        } else {
            s_reftrack(i) = s_reftrack(i - 1) + (reftrack.col(i) - reftrack.col(i - 1)).norm();
        }
    }
    
    // For each path point, find closest reference track point
    for (int i = 0; i < n_path_points; ++i) {
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;
        
        for (int j = 0; j < n_reftrack_points; ++j) {
            double dist = (path.col(i) - reftrack.col(j)).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = j;
            }
        }
        
        s_interp(i) = s_reftrack(closest_idx);
    }
    
    return s_interp;
}

} // namespace trajectory_planning_helpers