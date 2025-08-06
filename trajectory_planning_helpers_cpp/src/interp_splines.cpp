#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>
#include <stdexcept>

namespace trajectory_planning_helpers {

std::tuple<Matrix2Xd, VectorXd, VectorXd, VectorXd> interp_splines(
    const MatrixXd& coeffs_x,
    const MatrixXd& coeffs_y,
    int incl_last_point,
    double stepsize_approx) {
    
    int no_splines = coeffs_x.rows();
    
    if (coeffs_x.rows() != coeffs_y.rows()) {
        throw std::runtime_error("coeffs_x and coeffs_y must have same number of rows!");
    }
    
    if (coeffs_x.cols() != 4 || coeffs_y.cols() != 4) {
        throw std::runtime_error("Coefficient matrices must have 4 columns!");
    }
    
    // Calculate approximate number of interpolation points per spline
    std::vector<int> no_interp_points(no_splines);
    int total_points = 0;
    
    for (int i = 0; i < no_splines; ++i) {
        // Calculate spline length approximation
        int n_samples = 100;
        double length = 0.0;
        
        for (int j = 0; j < n_samples; ++j) {
            double t1 = double(j) / n_samples;
            double t2 = double(j + 1) / n_samples;
            
            // Calculate derivatives at t1 and t2
            double dx1 = coeffs_x(i, 1) + 2.0 * coeffs_x(i, 2) * t1 + 3.0 * coeffs_x(i, 3) * t1 * t1;
            double dy1 = coeffs_y(i, 1) + 2.0 * coeffs_y(i, 2) * t1 + 3.0 * coeffs_y(i, 3) * t1 * t1;
            double dx2 = coeffs_x(i, 1) + 2.0 * coeffs_x(i, 2) * t2 + 3.0 * coeffs_x(i, 3) * t2 * t2;
            double dy2 = coeffs_y(i, 1) + 2.0 * coeffs_y(i, 2) * t2 + 3.0 * coeffs_y(i, 3) * t2 * t2;
            
            double speed_avg = 0.5 * (std::sqrt(dx1*dx1 + dy1*dy1) + std::sqrt(dx2*dx2 + dy2*dy2));
            length += speed_avg / n_samples;
        }
        
        no_interp_points[i] = std::max(1, int(std::ceil(length / stepsize_approx)));
        total_points += no_interp_points[i];
    }
    
    if (incl_last_point > 0) {
        total_points += 1;
    }
    
    // Interpolate splines
    Matrix2Xd path_interp(2, total_points);
    VectorXd spline_inds(total_points);
    VectorXd t_values(total_points);
    VectorXd s_values(total_points);
    
    int point_idx = 0;
    double s_current = 0.0;
    
    for (int i = 0; i < no_splines; ++i) {
        for (int j = 0; j < no_interp_points[i]; ++j) {
            double t = double(j) / no_interp_points[i];
            
            // Calculate position
            double x = coeffs_x(i, 0) + coeffs_x(i, 1) * t + coeffs_x(i, 2) * t * t + coeffs_x(i, 3) * t * t * t;
            double y = coeffs_y(i, 0) + coeffs_y(i, 1) * t + coeffs_y(i, 2) * t * t + coeffs_y(i, 3) * t * t * t;
            
            path_interp(0, point_idx) = x;
            path_interp(1, point_idx) = y;
            spline_inds(point_idx) = i;
            t_values(point_idx) = t;
            
            // Calculate arc length (approximate)
            if (point_idx > 0) {
                s_current += (path_interp.col(point_idx) - path_interp.col(point_idx - 1)).norm();
            }
            s_values(point_idx) = s_current;
            
            point_idx++;
        }
    }
    
    // Include last point if requested
    if (incl_last_point > 0) {
        double t = 1.0;
        int last_spline = no_splines - 1;
        
        double x = coeffs_x(last_spline, 0) + coeffs_x(last_spline, 1) * t + 
                   coeffs_x(last_spline, 2) * t * t + coeffs_x(last_spline, 3) * t * t * t;
        double y = coeffs_y(last_spline, 0) + coeffs_y(last_spline, 1) * t + 
                   coeffs_y(last_spline, 2) * t * t + coeffs_y(last_spline, 3) * t * t * t;
        
        path_interp(0, point_idx) = x;
        path_interp(1, point_idx) = y;
        spline_inds(point_idx) = last_spline;
        t_values(point_idx) = t;
        s_current += (path_interp.col(point_idx) - path_interp.col(point_idx - 1)).norm();
        s_values(point_idx) = s_current;
    }
    
    return std::make_tuple(path_interp, spline_inds, t_values, s_values);
}

} // namespace trajectory_planning_helpers