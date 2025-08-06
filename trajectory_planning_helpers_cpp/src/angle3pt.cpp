#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>

namespace trajectory_planning_helpers {

VectorXd angle3pt(const Matrix2Xd& points) {
    int n_points = points.cols();
    VectorXd angles(n_points);
    
    for (int i = 0; i < n_points; ++i) {
        Vector2d p1, p2, p3;
        
        if (i == 0) {
            p1 = points.col(n_points - 1);
            p2 = points.col(0);
            p3 = points.col(1);
        } else if (i == n_points - 1) {
            p1 = points.col(n_points - 2);
            p2 = points.col(n_points - 1);
            p3 = points.col(0);
        } else {
            p1 = points.col(i - 1);
            p2 = points.col(i);
            p3 = points.col(i + 1);
        }
        
        Vector2d v1 = p1 - p2;
        Vector2d v2 = p3 - p2;
        
        double dot_product = v1.dot(v2);
        double cross_product = v1(0) * v2(1) - v1(1) * v2(0);
        
        angles(i) = std::atan2(cross_product, dot_product);
    }
    
    return angles;
}

} // namespace trajectory_planning_helpers