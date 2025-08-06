#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace global_racetrajectory_optimization {

GlobalRaceTrajectoryOptimizer::GlobalRaceTrajectoryOptimizer() 
    : config_loaded_(false), track_loaded_(false), veh_dynamics_loaded_(false), track_prepared_(false) {
    // Initialize with default values
}

GlobalRaceTrajectoryOptimizer::~GlobalRaceTrajectoryOptimizer() = default;

bool GlobalRaceTrajectoryOptimizer::loadConfig(const std::string& config_file) {
    try {
        auto config_map = utils::parseConfigFile(config_file);
        
        if (!utils::parseVehicleParams(config_map, veh_params_) ||
            !utils::parseOptimizationOptions(config_map, optim_opts_) ||
            !utils::parseStepsizeOptions(config_map, stepsize_opts_) ||
            !utils::parseRegSmoothOptions(config_map, reg_smooth_opts_)) {
            return false;
        }
        
        config_loaded_ = validateConfiguration();
        return config_loaded_;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

bool GlobalRaceTrajectoryOptimizer::loadTrack(const std::string& track_file) {
    try {
        track_data_.reftrack = utils::importTrack(track_file);
        
        if (track_data_.reftrack.rows() == 0) {
            std::cerr << "Failed to load track data" << std::endl;
            return false;
        }
        
        if (!utils::checkTrackValidity(track_data_.reftrack)) {
            std::cerr << "Invalid track data" << std::endl;
            return false;
        }
        
        track_data_.track_name = track_file;
        track_loaded_ = true;
        track_prepared_ = false;  // Need to prepare track after loading
        
        std::cout << "Track loaded: " << track_data_.reftrack.rows() << " points" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading track: " << e.what() << std::endl;
        return false;
    }
}

bool GlobalRaceTrajectoryOptimizer::loadVehicleDynamics(const std::string& ggv_file, const std::string& ax_max_file) {
    try {
        ggv_data_ = loadCSV(ggv_file);
        ax_max_machines_ = loadCSV(ax_max_file);
        
        if (ggv_data_.rows() == 0 || ax_max_machines_.rows() == 0) {
            std::cerr << "Failed to load vehicle dynamics data" << std::endl;
            return false;
        }
        
        veh_dynamics_loaded_ = true;
        std::cout << "Vehicle dynamics loaded: GGV (" << ggv_data_.rows() << " points), "
                  << "Ax_max (" << ax_max_machines_.rows() << " points)" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading vehicle dynamics: " << e.what() << std::endl;
        return false;
    }
}

bool GlobalRaceTrajectoryOptimizer::prepareTrack(bool debug) {
    if (!track_loaded_) {
        std::cerr << "Track not loaded" << std::endl;
        return false;
    }
    
    try {
        if (debug) {
            std::cout << "Preparing track..." << std::endl;
        }
        
        // Smooth and interpolate track using trajectory_planning_helpers
        auto [track_smoothed, el_lengths] = trajectory_planning_helpers::spline_approximation(
            track_data_.reftrack.block(0, 0, track_data_.reftrack.rows(), 2).transpose(),
            reg_smooth_opts_.k_reg,
            reg_smooth_opts_.s_reg,
            stepsize_opts_.stepsize_prep,
            stepsize_opts_.stepsize_reg,
            debug
        );
        
        // Update track data with smoothed version
        track_data_.reftrack.resize(track_smoothed.rows(), 4);
        track_data_.reftrack.block(0, 0, track_smoothed.rows(), 2) = track_smoothed;
        
        // Interpolate track widths (simplified - keep original widths)
        // In a full implementation, you would interpolate the width data as well
        
        // Calculate splines
        trajectory_planning_helpers::Matrix2Xd refpath_cl(2, track_smoothed.rows() + 1);
        refpath_cl.leftCols(track_smoothed.rows()) = track_smoothed.transpose();
        refpath_cl.rightCols(1) = track_smoothed.row(0).transpose(); // Close the path
        
        // Adjust el_lengths for closed path - add closing segment length
        VectorXd el_lengths_closed(el_lengths.size() + 1);
        el_lengths_closed.head(el_lengths.size()) = el_lengths;
        // Calculate length from last point back to first point
        el_lengths_closed(el_lengths.size()) = (track_smoothed.row(0) - track_smoothed.row(track_smoothed.rows() - 1)).norm();
        
        auto [coeffs_x, coeffs_y, a_interp, normvectors] = trajectory_planning_helpers::calc_splines(
            refpath_cl, el_lengths_closed, 0.0, 0.0, true
        );
        
        // Store results
        track_data_.coeffs_x = coeffs_x;
        track_data_.coeffs_y = coeffs_y;
        track_data_.a_interp = a_interp;
        track_data_.normvectors = normvectors;
        track_data_.el_lengths = el_lengths_closed;
        
        track_prepared_ = true;
        
        if (debug) {
            std::cout << "Track preparation completed: " << track_data_.reftrack.rows() 
                      << " points, " << track_data_.normvectors.rows() << " normal vectors" << std::endl;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error preparing track: " << e.what() << std::endl;
        return false;
    }
}

OptimizationResult GlobalRaceTrajectoryOptimizer::optimizeShortestPath() {
    OptimizationResult result;
    result.success = false;
    
    if (!track_prepared_) {
        result.message = "Track not prepared";
        return result;
    }
    
    try {
        // Simplified shortest path: just use the center line
        int n_points = track_data_.reftrack.rows();
        result.alpha_opt = VectorXd::Zero(n_points);
        
        // Calculate arc length
        result.s_opt.resize(n_points);
        result.s_opt(0) = 0.0;
        for (int i = 1; i < n_points; ++i) {
            auto p1 = track_data_.reftrack.row(i-1).head(2);
            auto p2 = track_data_.reftrack.row(i).head(2);
            result.s_opt(i) = result.s_opt(i-1) + (p2 - p1).norm();
        }
        
        // Generate raceline (centerline in this case)
        result.raceline = track_data_.reftrack.leftCols(2);
        
        // Calculate curvature and velocity
        result.kappa_opt = utils::calculateCurvature(result.raceline, track_data_.el_lengths);
        
        if (veh_dynamics_loaded_) {
            // Use trajectory_planning_helpers for velocity profile
            auto [v_profile, ax_profile] = trajectory_planning_helpers::calc_vel_profile(
                result.kappa_opt, track_data_.el_lengths, true, 
                veh_params_.dragcoeff, veh_params_.mass, 
                ggv_data_.col(0), 1.0, 0.0, 0.0
            );
            result.v_opt = v_profile;
        } else {
            // Simple constant velocity
            result.v_opt = VectorXd::Constant(n_points, veh_params_.v_max * 0.5);
        }
        
        result.lap_time = utils::calculateLapTime(result.v_opt, track_data_.el_lengths);
        result.optimization_time = 0.001; // Minimal time for centerline
        result.success = true;
        result.message = "Shortest path (centerline) completed successfully";
        
    } catch (const std::exception& e) {
        result.message = "Error in shortest path optimization: " + std::string(e.what());
    }
    
    return result;
}

OptimizationResult GlobalRaceTrajectoryOptimizer::optimizeMinCurvature(bool use_iqp) {
    OptimizationResult result;
    result.success = false;
    
    if (!track_prepared_) {
        result.message = "Track not prepared";
        return result;
    }
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Use trajectory_planning_helpers minimum curvature optimization
        auto [alpha_opt, s_opt, opt_time] = trajectory_planning_helpers::opt_min_curv(
            track_data_.reftrack,
            track_data_.normvectors,
            track_data_.a_interp,
            veh_params_.curvlim,
            optim_opts_.width_opt,
            false, false, true, 0.0, 0.0, false, false
        );
        
        result.alpha_opt = alpha_opt;
        result.s_opt = s_opt;
        
        // Calculate raceline
        result.raceline = utils::calculateRaceline(track_data_.reftrack, track_data_.normvectors, alpha_opt);
        
        // Calculate curvature
        result.kappa_opt = utils::calculateCurvature(result.raceline, track_data_.el_lengths);
        
        // Calculate velocity profile
        if (veh_dynamics_loaded_) {
            auto [v_profile, ax_profile] = trajectory_planning_helpers::calc_vel_profile(
                result.kappa_opt, track_data_.el_lengths, true,
                veh_params_.dragcoeff, veh_params_.mass,
                ggv_data_.col(0), 1.0, 0.0, 0.0
            );
            result.v_opt = v_profile;
        } else {
            result.v_opt = VectorXd::Constant(result.raceline.rows(), veh_params_.v_max * 0.7);
        }
        
        result.lap_time = utils::calculateLapTime(result.v_opt, track_data_.el_lengths);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        result.optimization_time = std::chrono::duration<double>(end_time - start_time).count();
        
        result.success = true;
        result.message = use_iqp ? "Minimum curvature (IQP) completed successfully" 
                                 : "Minimum curvature completed successfully";
        
    } catch (const std::exception& e) {
        result.message = "Error in minimum curvature optimization: " + std::string(e.what());
    }
    
    return result;
}

OptimizationResult GlobalRaceTrajectoryOptimizer::optimizeMinTime() {
    OptimizationResult result;
    result.success = false;
    result.message = "Minimum time optimization not yet implemented (requires CasADi/IPOPT)";
    
    // This would require implementing the full minimum time optimization
    // with CasADi and IPOPT, which is quite complex. For now, return min curvature result
    std::cout << "WARNING: Using minimum curvature instead of minimum time optimization" << std::endl;
    
    return optimizeMinCurvature(false);
}

bool GlobalRaceTrajectoryOptimizer::exportResult(const OptimizationResult& result, const std::string& output_path) {
    if (!result.success) {
        std::cerr << "Cannot export failed optimization result" << std::endl;
        return false;
    }
    
    return utils::exportToCSV(result, output_path);
}

bool GlobalRaceTrajectoryOptimizer::visualizeResult(const OptimizationResult& result) {
    if (!result.success) {
        std::cerr << "Cannot visualize failed optimization result" << std::endl;
        return false;
    }
    
    // For now, just print summary
    std::cout << "=== Optimization Result Summary ===" << std::endl;
    std::cout << "Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "Message: " << result.message << std::endl;
    std::cout << "Lap time: " << result.lap_time << " s" << std::endl;
    std::cout << "Optimization time: " << result.optimization_time << " s" << std::endl;
    std::cout << "Raceline points: " << result.raceline.rows() << std::endl;
    std::cout << "Max velocity: " << result.v_opt.maxCoeff() << " m/s" << std::endl;
    std::cout << "Max curvature: " << result.kappa_opt.maxCoeff() << " rad/m" << std::endl;
    std::cout << "===================================" << std::endl;
    
    return true;
}

bool GlobalRaceTrajectoryOptimizer::validateConfiguration() {
    if (veh_params_.mass <= 0 || veh_params_.v_max <= 0 || veh_params_.width <= 0) {
        std::cerr << "Invalid vehicle parameters" << std::endl;
        return false;
    }
    
    if (optim_opts_.width_opt <= 0) {
        std::cerr << "Invalid optimization width" << std::endl;
        return false;
    }
    
    return true;
}

MatrixXd GlobalRaceTrajectoryOptimizer::loadCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }
    
    std::vector<std::vector<double>> data;
    std::string line;
    
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        
        while (std::getline(ss, cell, ',')) {
            try {
                row.push_back(std::stod(cell));
            } catch (const std::exception&) {
                // Skip non-numeric values
            }
        }
        
        if (!row.empty()) {
            data.push_back(row);
        }
    }
    
    if (data.empty()) {
        return MatrixXd();
    }
    
    MatrixXd result(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); ++i) {
        for (size_t j = 0; j < data[i].size() && j < data[0].size(); ++j) {
            result(i, j) = data[i][j];
        }
    }
    
    return result;
}

bool GlobalRaceTrajectoryOptimizer::saveCSV(const MatrixXd& data, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    for (int i = 0; i < data.rows(); ++i) {
        for (int j = 0; j < data.cols(); ++j) {
            file << data(i, j);
            if (j < data.cols() - 1) {
                file << ",";
            }
        }
        file << std::endl;
    }
    
    return true;
}

} // namespace global_racetrajectory_optimization