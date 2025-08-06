#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace global_racetrajectory_optimization::utils {

MatrixXd importTrack(const std::string& filename, bool flip_track) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open track file: " << filename << std::endl;
        return MatrixXd();
    }
    
    std::vector<std::vector<double>> data;
    std::string line;
    bool skip_header = true;
    
    while (std::getline(file, line)) {
        // Skip header line if it contains non-numeric data
        if (skip_header && (line.find_first_of("0123456789-") != 0)) {
            continue;
        }
        skip_header = false;
        
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        
        while (std::getline(ss, cell, ',')) {
            try {
                // Remove whitespace
                cell.erase(0, cell.find_first_not_of(" \t"));
                cell.erase(cell.find_last_not_of(" \t") + 1);
                
                if (!cell.empty()) {
                    row.push_back(std::stod(cell));
                }
            } catch (const std::exception&) {
                std::cerr << "Warning: Could not parse cell: " << cell << std::endl;
            }
        }
        
        // Ensure we have at least 4 columns (x, y, w_tr_right, w_tr_left)
        if (row.size() >= 4) {
            data.push_back(row);
        } else if (row.size() == 2) {
            // If only x, y provided, add default track widths
            row.push_back(3.0); // Default right width
            row.push_back(3.0); // Default left width
            data.push_back(row);
        }
    }
    
    if (data.empty()) {
        std::cerr << "No valid track data found in file: " << filename << std::endl;
        return MatrixXd();
    }
    
    // Convert to Eigen matrix
    MatrixXd track(data.size(), 4);
    for (size_t i = 0; i < data.size(); ++i) {
        for (size_t j = 0; j < 4 && j < data[i].size(); ++j) {
            track(i, j) = data[i][j];
        }
        // Fill missing columns with default values
        if (data[i].size() < 4) {
            for (size_t j = data[i].size(); j < 4; ++j) {
                track(i, j) = 3.0; // Default track width
            }
        }
    }
    
    // Flip track if requested
    if (flip_track) {
        MatrixXd flipped_track(track.rows(), track.cols());
        for (int i = 0; i < track.rows(); ++i) {
            flipped_track.row(i) = track.row(track.rows() - 1 - i);
            // Swap left and right track widths
            std::swap(flipped_track(i, 2), flipped_track(i, 3));
        }
        track = flipped_track;
    }
    
    std::cout << "Track imported: " << track.rows() << " points" << std::endl;
    return track;
}

bool checkTrackValidity(const MatrixXd& track) {
    if (track.rows() < 3) {
        std::cerr << "Track must have at least 3 points" << std::endl;
        return false;
    }
    
    if (track.cols() < 4) {
        std::cerr << "Track must have at least 4 columns (x, y, w_tr_right, w_tr_left)" << std::endl;
        return false;
    }
    
    // Check for reasonable track widths
    for (int i = 0; i < track.rows(); ++i) {
        if (track(i, 2) <= 0 || track(i, 3) <= 0) {
            std::cerr << "Invalid track width at point " << i << std::endl;
            return false;
        }
        
        if (track(i, 2) > 50 || track(i, 3) > 50) {
            std::cerr << "Warning: Very large track width at point " << i << std::endl;
        }
    }
    
    // Check for closed track (optional warning)
    Vector2d start_point = track.row(0).head(2);
    Vector2d end_point = track.row(track.rows() - 1).head(2);
    double distance = (end_point - start_point).norm();
    
    if (distance > 10.0) {
        std::cout << "Note: Track appears to be open (start-end distance: " << distance << " m)" << std::endl;
    }
    
    return true;
}

MatrixXd setNewStartPoint(const MatrixXd& track, const Vector2d& new_start) {
    if (track.rows() == 0) {
        return track;
    }
    
    // Find closest point to new_start
    int closest_idx = 0;
    double min_dist = (track.row(0).head(2) - new_start.transpose()).norm();
    
    for (int i = 1; i < track.rows(); ++i) {
        double dist = (track.row(i).head(2) - new_start.transpose()).norm();
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    if (closest_idx == 0) {
        return track; // Already at the desired start point
    }
    
    // Reorder track to start at closest_idx
    MatrixXd reordered_track(track.rows(), track.cols());
    
    for (int i = 0; i < track.rows(); ++i) {
        int src_idx = (closest_idx + i) % track.rows();
        reordered_track.row(i) = track.row(src_idx);
    }
    
    std::cout << "New start point set: moved from index 0 to " << closest_idx << std::endl;
    return reordered_track;
}

MatrixXd calculateRaceline(const MatrixXd& reftrack, const MatrixXd& normvectors, const VectorXd& alpha) {
    if (reftrack.rows() != normvectors.rows() || reftrack.rows() != alpha.size()) {
        throw std::runtime_error("Dimension mismatch in raceline calculation");
    }
    
    MatrixXd raceline(reftrack.rows(), 2);
    
    for (int i = 0; i < reftrack.rows(); ++i) {
        Vector2d ref_point = reftrack.row(i).head(2);
        Vector2d normal = normvectors.row(i);
        
        raceline.row(i) = (ref_point + alpha(i) * normal).transpose();
    }
    
    return raceline;
}

VectorXd calculateCurvature(const MatrixXd& raceline, const VectorXd& el_lengths, bool closed) {
    int n_points = raceline.rows();
    VectorXd curvature(n_points);
    
    // Use trajectory_planning_helpers for curvature calculation
    trajectory_planning_helpers::Matrix2Xd path_2xn = raceline.transpose();
    
    try {
        auto [psi, kappa] = trajectory_planning_helpers::calc_head_curv_num(
            path_2xn, el_lengths, closed, 1.0, 1.0, 2.0, 2.0, true
        );
        
        curvature = kappa;
        
    } catch (const std::exception& e) {
        std::cerr << "Warning: Could not calculate curvature using TPH, using fallback method" << std::endl;
        
        // Fallback: simple finite difference approximation
        for (int i = 0; i < n_points; ++i) {
            Vector2d p1, p2, p3;
            
            if (closed) {
                p1 = raceline.row((i - 1 + n_points) % n_points);
                p2 = raceline.row(i);
                p3 = raceline.row((i + 1) % n_points);
            } else {
                if (i == 0) {
                    p1 = raceline.row(0);
                    p2 = raceline.row(1);
                    p3 = raceline.row(2);
                } else if (i == n_points - 1) {
                    p1 = raceline.row(n_points - 3);
                    p2 = raceline.row(n_points - 2);
                    p3 = raceline.row(n_points - 1);
                } else {
                    p1 = raceline.row(i - 1);
                    p2 = raceline.row(i);
                    p3 = raceline.row(i + 1);
                }
            }
            
            // Calculate curvature using three-point method
            Vector2d v1 = p2 - p1;
            Vector2d v2 = p3 - p2;
            
            double cross = v1(0) * v2(1) - v1(1) * v2(0);
            double dot = v1.dot(v2);
            double norm_product = v1.norm() * v2.norm();
            
            if (norm_product > 1e-10) {
                double angle_change = std::atan2(cross, dot);
                double arc_length = (v1.norm() + v2.norm()) * 0.5;
                curvature(i) = (arc_length > 1e-10) ? angle_change / arc_length : 0.0;
            } else {
                curvature(i) = 0.0;
            }
        }
    }
    
    return curvature;
}

double calculateLapTime(const VectorXd& v_profile, const VectorXd& el_lengths) {
    if (v_profile.size() != el_lengths.size() + 1 && v_profile.size() != el_lengths.size()) {
        std::cerr << "Warning: Velocity profile and element lengths size mismatch" << std::endl;
    }
    
    double total_time = 0.0;
    int n_segments = std::min(static_cast<int>(v_profile.size() - 1), static_cast<int>(el_lengths.size()));
    
    for (int i = 0; i < n_segments; ++i) {
        double v_avg = 0.5 * (v_profile(i) + v_profile(i + 1));
        if (v_avg > 1e-6) {
            total_time += el_lengths(i) / v_avg;
        }
    }
    
    return total_time;
}

bool exportToCSV(const OptimizationResult& result, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Write header
    file << "x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,s_m" << std::endl;
    
    int n_points = result.raceline.rows();
    
    for (int i = 0; i < n_points; ++i) {
        file << result.raceline(i, 0) << ","
             << result.raceline(i, 1) << ","
             << "0.0,"  // psi (would need to calculate from raceline)
             << result.kappa_opt(i) << ","
             << result.v_opt(i) << ","
             << "0.0,"  // ax (would need to calculate from velocity profile)
             << result.s_opt(i) << std::endl;
    }
    
    return true;
}

bool exportToLTPL(const OptimizationResult& result, const std::string& filename) {
    // LTPL format export (simplified)
    return exportToCSV(result, filename);
}

} // namespace global_racetrajectory_optimization::utils