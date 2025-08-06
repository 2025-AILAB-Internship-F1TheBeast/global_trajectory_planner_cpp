#pragma once

#include <Eigen/Dense>
#include <vector>
#include <tuple>

namespace trajectory_planning_helpers {

// Type aliases for better readability
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Matrix2Xd = Eigen::Matrix2Xd;
using Vector2d = Eigen::Vector2d;

// Spline calculation functions
std::tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> calc_splines(
    const Matrix2Xd& path,
    const VectorXd& el_lengths = VectorXd(),
    double psi_s = 0.0,
    double psi_e = 0.0,
    bool use_dist_scaling = true
);

// Curvature calculation functions
std::tuple<VectorXd, VectorXd> calc_head_curv_num(
    const Matrix2Xd& path,
    const VectorXd& el_lengths,
    bool is_closed,
    double stepsize_psi_preview = 1.0,
    double stepsize_psi_review = 1.0,
    double stepsize_curv_preview = 2.0,
    double stepsize_curv_review = 2.0,
    bool calc_curv = true
);

// Spline interpolation
std::tuple<Matrix2Xd, VectorXd, VectorXd, VectorXd> interp_splines(
    const MatrixXd& coeffs_x,
    const MatrixXd& coeffs_y,
    int incl_last_point = 0,
    double stepsize_approx = 1.0
);

// Normal vector calculation
MatrixXd calc_normal_vectors(const VectorXd& psi);

// Tangent vector calculation  
MatrixXd calc_tangent_vectors(const VectorXd& psi);

// Angle normalization
VectorXd normalize_psi(const VectorXd& psi);
double normalize_psi(double psi);

// 3-point angle calculation
VectorXd angle3pt(const Matrix2Xd& points);

// Optimization functions
std::tuple<VectorXd, VectorXd, double> opt_min_curv(
    const MatrixXd& reftrack,
    const MatrixXd& normvectors,
    const MatrixXd& A,
    double kappa_bound,
    double w_veh,
    bool print_debug = false,
    bool plot_debug = false,
    bool closed = true,
    double psi_s = 0.0,
    double psi_e = 0.0,
    bool fix_s = false,
    bool fix_e = false
);

// Velocity profile calculation
std::tuple<VectorXd, VectorXd> calc_vel_profile(
    const VectorXd& kappa,
    const VectorXd& el_lengths,
    bool closed,
    double drag_coeff,
    double m_veh,
    const VectorXd& ggv,
    double mu = 1.0,
    double v_start = 0.0,
    double v_end = 0.0
);

// Path matching functions
VectorXd path_matching_global(
    const Matrix2Xd& path,
    const Matrix2Xd& reftrack,
    const VectorXd& el_lengths_reftrack
);

std::tuple<int, double> path_matching_local(
    const Vector2d& pos_est,
    const Matrix2Xd& reftrack,
    int s_ind_last_guess,
    const VectorXd& el_lengths = VectorXd()
);

// Spline approximation
std::tuple<MatrixXd, VectorXd> spline_approximation(
    const Matrix2Xd& track,
    int k_reg = 3,
    double s_reg = 10.0,
    int stepsize_prep = 1,
    double stepsize_reg = 1.5,
    bool debug = false
);

} // namespace trajectory_planning_helpers