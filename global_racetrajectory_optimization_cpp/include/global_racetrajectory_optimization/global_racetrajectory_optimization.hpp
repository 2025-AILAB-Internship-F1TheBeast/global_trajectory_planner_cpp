#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <memory>

namespace global_racetrajectory_optimization {

// Type aliases
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Matrix2Xd = Eigen::Matrix2Xd;
using Vector2d = Eigen::Vector2d;

// Forward declarations
struct VehicleParameters;
struct OptimizationOptions;
struct TrackData;
struct OptimizationResult;

// Enums
enum class OptimizationType {
    SHORTEST_PATH,
    MIN_CURVATURE,
    MIN_CURVATURE_IQP,
    MIN_TIME
};

// Configuration and parameter structures
struct StepsizeOptions {
    double stepsize_prep = 1.0;
    double stepsize_reg = 3.0;
    double stepsize_interp_after_opt = 2.0;
};

struct RegSmoothOptions {
    int k_reg = 3;
    double s_reg = 10.0;
};

struct CurvCalcOptions {
    double d_preview_curv = 2.0;
    double d_review_curv = 2.0;
    double d_preview_head = 1.0;
    double d_review_head = 1.0;
};

struct VehicleParameters {
    double v_max = 70.0;         // [m/s] maximal vehicle speed
    double length = 4.7;         // [m] vehicle length
    double width = 2.0;          // [m] vehicle width
    double mass = 1200.0;        // [kg] vehicle mass
    double dragcoeff = 0.75;     // [kg*m2/m3] drag coefficient
    double curvlim = 0.12;       // [rad/m] curvature limit
    double g = 9.81;             // [N/kg] gravity acceleration
};

struct OptimizationOptions {
    double width_opt = 3.4;      // [m] vehicle width for optimization
    int iqp_iters_min = 3;       // [-] minimum IQP iterations
    double iqp_curverror_allowed = 0.01;  // [rad/m] allowed curvature error
    double penalty_delta = 10.0; // [-] penalty for delta derivative
    double penalty_F = 0.01;     // [-] penalty for F derivative  
    double mue = 1.0;            // [-] friction coefficient
    int n_gauss = 5;             // [-] gaussian basis functions
    double dn = 0.25;            // [m] distance for friction extraction
    bool limit_energy = false;   // energy consumption limit
    double energy_limit = 2.0;   // [kWh/lap] energy limit
    bool safe_traj = false;      // safe trajectories flag
    double w_tr_reopt = 2.0;     // [m] track width for reoptimization
    double w_veh_reopt = 1.6;    // [m] vehicle width for reoptimization
    int step_non_reg = 0;        // [-] non-regular sampling step
    double eps_kappa = 1e-3;     // [rad/m] curvature threshold
};

struct TrackData {
    MatrixXd reftrack;           // [x, y, w_tr_right, w_tr_left]
    MatrixXd coeffs_x;           // spline coefficients x
    MatrixXd coeffs_y;           // spline coefficients y
    MatrixXd normvectors;        // normalized normal vectors
    MatrixXd a_interp;           // spline interpolation matrix
    VectorXd el_lengths;         // element lengths
    std::string track_name;      // track identifier
};

struct OptimizationResult {
    VectorXd alpha_opt;          // optimal lateral shift
    VectorXd s_opt;              // arc length coordinates
    VectorXd v_opt;              // optimal velocity profile
    VectorXd kappa_opt;          // optimal curvature profile
    MatrixXd raceline;           // optimal raceline [x, y]
    double lap_time;             // total lap time
    double optimization_time;    // optimization duration
    bool success;                // optimization success flag
    std::string message;         // result message
};

// Main optimization class
class GlobalRaceTrajectoryOptimizer {
public:
    GlobalRaceTrajectoryOptimizer();
    ~GlobalRaceTrajectoryOptimizer();

    // Configuration
    bool loadConfig(const std::string& config_file);
    bool loadTrack(const std::string& track_file);
    bool loadVehicleDynamics(const std::string& ggv_file, const std::string& ax_max_file);

    // Track preparation
    bool prepareTrack(bool debug = true);
    
    // Optimization methods
    OptimizationResult optimizeShortestPath();
    OptimizationResult optimizeMinCurvature(bool use_iqp = false);
    OptimizationResult optimizeMinTime();
    
    // Utility functions
    bool exportResult(const OptimizationResult& result, const std::string& output_path);
    bool visualizeResult(const OptimizationResult& result);
    
    // Getters
    const TrackData& getTrackData() const { return track_data_; }
    const VehicleParameters& getVehicleParams() const { return veh_params_; }
    const OptimizationOptions& getOptimizationOptions() const { return optim_opts_; }

private:
    // Member variables
    TrackData track_data_;
    VehicleParameters veh_params_;
    OptimizationOptions optim_opts_;
    StepsizeOptions stepsize_opts_;
    RegSmoothOptions reg_smooth_opts_;
    CurvCalcOptions curv_calc_opts_;
    
    MatrixXd ggv_data_;          // GGV diagram data
    MatrixXd ax_max_machines_;   // Machine acceleration limits
    
    bool config_loaded_;
    bool track_loaded_;
    bool veh_dynamics_loaded_;
    bool track_prepared_;
    
    // Helper methods
    bool validateConfiguration();
    bool interpolateTrack();
    bool calculateSplines();
    MatrixXd loadCSV(const std::string& filename);
    bool saveCSV(const MatrixXd& data, const std::string& filename);
};

// Standalone utility functions
namespace utils {
    
    // Configuration parsing
    std::map<std::string, std::string> parseConfigFile(const std::string& filename);
    bool parseVehicleParams(const std::map<std::string, std::string>& config, VehicleParameters& params);
    bool parseOptimizationOptions(const std::map<std::string, std::string>& config, OptimizationOptions& opts);
    bool parseStepsizeOptions(const std::map<std::string, std::string>& config, StepsizeOptions& opts);
    bool parseRegSmoothOptions(const std::map<std::string, std::string>& config, RegSmoothOptions& opts);
    
    // Track utilities
    MatrixXd importTrack(const std::string& filename, bool flip_track = false);
    bool checkTrackValidity(const MatrixXd& track);
    MatrixXd setNewStartPoint(const MatrixXd& track, const Vector2d& new_start);
    
    // Result processing
    MatrixXd calculateRaceline(const MatrixXd& reftrack, const MatrixXd& normvectors, const VectorXd& alpha);
    VectorXd calculateCurvature(const MatrixXd& raceline, const VectorXd& el_lengths, bool closed = true);
    double calculateLapTime(const VectorXd& v_profile, const VectorXd& el_lengths);
    
    // Export utilities
    bool exportToCSV(const OptimizationResult& result, const std::string& filename);
    bool exportToLTPL(const OptimizationResult& result, const std::string& filename);
    
} // namespace utils

} // namespace global_racetrajectory_optimization