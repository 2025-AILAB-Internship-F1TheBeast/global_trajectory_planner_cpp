#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>
#include <stdexcept>

namespace trajectory_planning_helpers {

std::tuple<VectorXd, VectorXd> calc_head_curv_num(
    const Matrix2Xd& path,
    const VectorXd& el_lengths,
    bool is_closed,
    double stepsize_psi_preview,
    double stepsize_psi_review,
    double stepsize_curv_preview,
    double stepsize_curv_review,
    bool calc_curv) {
    
    int n_points = path.cols();
    
    // Check inputs
    if (is_closed && n_points != el_lengths.size()) {
        throw std::runtime_error("path and el_lengths must have the same length!");
    } else if (!is_closed && n_points != el_lengths.size() + 1) {
        throw std::runtime_error("path must have the length of el_lengths + 1!");
    }
    
    VectorXd psi(n_points);
    VectorXd kappa(n_points);
    
    if (is_closed) {
        // CLOSED PATH CASE
        
        // Calculate preview/review distances
        double avg_el_length = el_lengths.mean();
        int ind_step_preview_psi = std::max(1, int(std::round(stepsize_psi_preview / avg_el_length)));
        int ind_step_review_psi = std::max(1, int(std::round(stepsize_psi_review / avg_el_length)));
        int ind_step_preview_curv = std::max(1, int(std::round(stepsize_curv_preview / avg_el_length)));
        int ind_step_review_curv = std::max(1, int(std::round(stepsize_curv_review / avg_el_length)));
        
        int steps_tot_psi = ind_step_preview_psi + ind_step_review_psi;
        int steps_tot_curv = ind_step_preview_curv + ind_step_review_curv;
        
        // HEADING CALCULATION
        // Create extended path for boundary handling
        Matrix2Xd path_temp(2, n_points + steps_tot_psi);
        
        // Add review points at the beginning
        for (int i = 0; i < ind_step_review_psi; ++i) {
            path_temp.col(i) = path.col(n_points - ind_step_review_psi + i);
        }
        
        // Add original path
        path_temp.middleCols(ind_step_review_psi, n_points) = path;
        
        // Add preview points at the end
        for (int i = 0; i < ind_step_preview_psi; ++i) {
            path_temp.col(ind_step_review_psi + n_points + i) = path.col(i);
        }
        
        // Calculate tangent vectors
        for (int i = 0; i < n_points; ++i) {
            Vector2d tangvec = path_temp.col(i + steps_tot_psi) - path_temp.col(i);
            psi(i) = std::atan2(tangvec(1), tangvec(0)) - M_PI / 2.0;
        }
        
        // Normalize psi
        psi = normalize_psi(psi);
        
        // CURVATURE CALCULATION
        if (calc_curv) {
            // Extend psi array for curvature calculation
            VectorXd psi_temp(n_points + steps_tot_curv);
            
            for (int i = 0; i < ind_step_review_curv; ++i) {
                psi_temp(i) = psi(n_points - ind_step_review_curv + i);
            }
            
            psi_temp.segment(ind_step_review_curv, n_points) = psi;
            
            for (int i = 0; i < ind_step_preview_curv; ++i) {
                psi_temp(ind_step_review_curv + n_points + i) = psi(i);
            }
            
            // Calculate delta psi
            VectorXd delta_psi(n_points);
            for (int i = 0; i < n_points; ++i) {
                delta_psi(i) = normalize_psi(psi_temp(i + steps_tot_curv) - psi_temp(i));
            }
            
            // Calculate cumulative distances
            VectorXd s_points_cl(n_points + 1);
            s_points_cl(0) = 0.0;
            for (int i = 0; i < n_points; ++i) {
                s_points_cl(i + 1) = s_points_cl(i) + el_lengths(i);
            }
            
            VectorXd s_points = s_points_cl.head(n_points);
            
            // Calculate reverse cumulative distances
            VectorXd s_points_cl_reverse(n_points);
            double cumsum = 0.0;
            for (int i = n_points - 1; i >= 0; --i) {
                cumsum += el_lengths(i);
                s_points_cl_reverse(n_points - 1 - i) = -cumsum;
            }
            
            // Extend s_points for curvature calculation
            VectorXd s_points_temp(n_points + steps_tot_curv);
            
            for (int i = 0; i < ind_step_review_curv; ++i) {
                s_points_temp(i) = s_points_cl_reverse(ind_step_review_curv - 1 - i);
            }
            
            s_points_temp.segment(ind_step_review_curv, n_points) = s_points;
            
            double total_track_length = s_points_cl(n_points);
            for (int i = 0; i < ind_step_preview_curv; ++i) {
                s_points_temp(ind_step_review_curv + n_points + i) = total_track_length + s_points(i);
            }
            
            // Calculate curvature
            for (int i = 0; i < n_points; ++i) {
                double ds = s_points_temp(i + steps_tot_curv) - s_points_temp(i);
                kappa(i) = delta_psi(i) / ds;
            }
        } else {
            kappa.setZero();
        }
        
    } else {
        // UNCLOSED PATH CASE
        
        // HEADING CALCULATION
        Matrix2Xd tangvecs(2, n_points);
        
        // First point
        tangvecs.col(0) = path.col(1) - path.col(0);
        
        // Middle points
        for (int i = 1; i < n_points - 1; ++i) {
            tangvecs.col(i) = path.col(i + 1) - path.col(i - 1);
        }
        
        // Last point
        tangvecs.col(n_points - 1) = path.col(n_points - 1) - path.col(n_points - 2);
        
        // Calculate psi
        for (int i = 0; i < n_points; ++i) {
            psi(i) = std::atan2(tangvecs(1, i), tangvecs(0, i)) - M_PI / 2.0;
        }
        
        psi = normalize_psi(psi);
        
        // CURVATURE CALCULATION
        if (calc_curv) {
            VectorXd delta_psi(n_points);
            
            // First point
            delta_psi(0) = psi(1) - psi(0);
            
            // Middle points  
            for (int i = 1; i < n_points - 1; ++i) {
                delta_psi(i) = psi(i + 1) - psi(i - 1);
            }
            
            // Last point
            delta_psi(n_points - 1) = psi(n_points - 1) - psi(n_points - 2);
            
            delta_psi = normalize_psi(delta_psi);
            
            // Calculate curvature
            kappa(0) = delta_psi(0) / el_lengths(0);
            
            for (int i = 1; i < n_points - 1; ++i) {
                kappa(i) = delta_psi(i) / (el_lengths(i) + el_lengths(i - 1));
            }
            
            kappa(n_points - 1) = delta_psi(n_points - 1) / el_lengths(n_points - 2);
            
        } else {
            kappa.setZero();
        }
    }
    
    return std::make_tuple(psi, kappa);
}

} // namespace trajectory_planning_helpers