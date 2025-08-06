#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace trajectory_planning_helpers {

std::tuple<VectorXd, VectorXd> calc_vel_profile(
    const VectorXd& kappa,
    const VectorXd& el_lengths,
    bool closed,
    double drag_coeff,
    double m_veh,
    const VectorXd& ggv,
    double mu,
    double v_start,
    double v_end) {
    
    int n_points = kappa.size();
    
    // Check inputs
    if (closed && n_points != el_lengths.size()) {
        throw std::runtime_error("kappa and el_lengths must have the same length for closed trajectory!");
    } else if (!closed && n_points != el_lengths.size() + 1) {
        throw std::runtime_error("kappa must have length el_lengths + 1 for unclosed trajectory!");
    }
    
    VectorXd vx_profile(n_points);
    VectorXd ax_profile(n_points);
    
    // Default GGV values if not provided (simplified)
    double v_max = 50.0;  // 50 m/s max velocity
    double ax_max = 8.0;  // 8 m/s² max longitudinal acceleration
    double ay_max = 8.0;  // 8 m/s² max lateral acceleration
    
    if (ggv.size() >= 3) {
        v_max = ggv(0);
        ax_max = ggv(1); 
        ay_max = ggv(2);
    }
    
    // FORWARD PASS - Calculate velocity limits based on lateral acceleration
    for (int i = 0; i < n_points; ++i) {
        // Calculate maximum velocity based on lateral acceleration limit
        double v_max_lat;
        if (std::abs(kappa(i)) > 1e-6) {
            v_max_lat = std::sqrt(ay_max * mu / std::abs(kappa(i)));
        } else {
            v_max_lat = v_max;
        }
        
        vx_profile(i) = std::min(v_max, v_max_lat);
    }
    
    // Set start velocity for unclosed trajectory
    if (!closed && v_start > 0.0) {
        vx_profile(0) = v_start;
    }
    
    // BACKWARD PASS - Enforce deceleration limits
    if (!closed && n_points > 1) {
        // Set end velocity
        if (v_end > 0.0) {
            vx_profile(n_points - 1) = v_end;
        }
        
        // Backward pass
        for (int i = n_points - 2; i >= 0; --i) {
            double ds = (i < el_lengths.size()) ? el_lengths(i) : el_lengths(i - 1);
            
            // Calculate maximum velocity considering deceleration capability
            double v_next = vx_profile(i + 1);
            double drag_decel = drag_coeff * v_next * v_next / m_veh;
            double available_decel = ax_max + drag_decel;
            
            double v_max_decel = std::sqrt(v_next * v_next + 2.0 * available_decel * ds);
            vx_profile(i) = std::min(vx_profile(i), v_max_decel);
        }
    }
    
    // FORWARD PASS - Enforce acceleration limits
    for (int i = 1; i < n_points; ++i) {
        double ds = (i - 1 < el_lengths.size()) ? el_lengths(i - 1) : el_lengths(el_lengths.size() - 1);
        
        // Calculate maximum velocity considering acceleration capability
        double v_prev = vx_profile(i - 1);
        double drag_resist = drag_coeff * v_prev * v_prev / m_veh;
        double available_accel = ax_max - drag_resist;
        
        if (available_accel > 0.0) {
            double v_max_accel = std::sqrt(v_prev * v_prev + 2.0 * available_accel * ds);
            vx_profile(i) = std::min(vx_profile(i), v_max_accel);
        } else {
            // Cannot maintain velocity due to drag
            double v_drag_limited = std::sqrt(v_prev * v_prev - 2.0 * std::abs(available_accel) * ds);
            vx_profile(i) = std::min(vx_profile(i), std::max(0.0, v_drag_limited));
        }
    }
    
    // Calculate acceleration profile
    for (int i = 0; i < n_points; ++i) {
        if (i == 0) {
            if (n_points > 1) {
                double ds = (0 < el_lengths.size()) ? el_lengths(0) : 1.0;
                ax_profile(i) = (vx_profile(1) * vx_profile(1) - vx_profile(0) * vx_profile(0)) / (2.0 * ds);
            } else {
                ax_profile(i) = 0.0;
            }
        } else if (i == n_points - 1) {
            double ds = (i - 1 < el_lengths.size()) ? el_lengths(i - 1) : 1.0;
            ax_profile(i) = (vx_profile(i) * vx_profile(i) - vx_profile(i - 1) * vx_profile(i - 1)) / (2.0 * ds);
        } else {
            double ds1 = (i - 1 < el_lengths.size()) ? el_lengths(i - 1) : 1.0;
            double ds2 = (i < el_lengths.size()) ? el_lengths(i) : 1.0;
            double ax1 = (vx_profile(i) * vx_profile(i) - vx_profile(i - 1) * vx_profile(i - 1)) / (2.0 * ds1);
            double ax2 = (vx_profile(i + 1) * vx_profile(i + 1) - vx_profile(i) * vx_profile(i)) / (2.0 * ds2);
            ax_profile(i) = 0.5 * (ax1 + ax2);
        }
        
        // Subtract drag acceleration
        ax_profile(i) -= drag_coeff * vx_profile(i) * vx_profile(i) / m_veh;
    }
    
    return std::make_tuple(vx_profile, ax_profile);
}

} // namespace trajectory_planning_helpers