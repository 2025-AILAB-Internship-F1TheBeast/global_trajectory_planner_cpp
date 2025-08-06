#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>

using namespace global_racetrajectory_optimization;

int main(int argc, char* argv[]) {
    std::cout << "=== Global Race Trajectory Optimization C++ ===" << std::endl;
    
    // Parse command line arguments
    std::string config_file = "params/racecar.ini";
    std::string track_name = "berlin_2018";
    std::string opt_type = "mincurv";
    bool debug = true;
    
    if (argc > 1) track_name = argv[1];
    if (argc > 2) opt_type = argv[2];
    if (argc > 3) config_file = argv[3];
    
    try {
        // Create optimizer
        GlobalRaceTrajectoryOptimizer optimizer;
        
        // Load configuration
        std::cout << "Loading configuration from: " << config_file << std::endl;
        if (!optimizer.loadConfig(config_file)) {
            std::cerr << "Failed to load configuration!" << std::endl;
            return -1;
        }
        
        // Load track
        std::string track_file = "inputs/tracks/" + track_name + ".csv";
        std::cout << "Loading track: " << track_file << std::endl;
        if (!optimizer.loadTrack(track_file)) {
            std::cerr << "Failed to load track!" << std::endl;
            return -1;
        }
        
        // Load vehicle dynamics
        std::string ggv_file = "inputs/veh_dyn_info/ggv.csv";
        std::string ax_max_file = "inputs/veh_dyn_info/ax_max_machines.csv";
        
        std::cout << "Loading vehicle dynamics..." << std::endl;
        if (!optimizer.loadVehicleDynamics(ggv_file, ax_max_file)) {
            std::cout << "Warning: Could not load vehicle dynamics, using defaults" << std::endl;
        }
        
        // Prepare track
        std::cout << "Preparing track..." << std::endl;
        if (!optimizer.prepareTrack(debug)) {
            std::cerr << "Failed to prepare track!" << std::endl;
            return -1;
        }
        
        // Run optimization
        OptimizationResult result;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        std::cout << "Running " << opt_type << " optimization..." << std::endl;
        
        if (opt_type == "shortest_path") {
            result = optimizer.optimizeShortestPath();
        } else if (opt_type == "mincurv") {
            result = optimizer.optimizeMinCurvature(false);
        } else if (opt_type == "mincurv_iqp") {
            result = optimizer.optimizeMinCurvature(true);
        } else if (opt_type == "mintime") {
            result = optimizer.optimizeMinTime();
        } else {
            std::cerr << "Unknown optimization type: " << opt_type << std::endl;
            std::cout << "Available types: shortest_path, mincurv, mincurv_iqp, mintime" << std::endl;
            return -1;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double>(end_time - start_time).count();
        
        // Display results
        std::cout << std::endl;
        if (result.success) {
            optimizer.visualizeResult(result);
            std::cout << "Total execution time: " << total_time << " s" << std::endl;
            
            // Export results
            std::string output_file = "outputs/" + track_name + "_" + opt_type + "_traj.csv";
            
            // Create output directory if it doesn't exist
            std::filesystem::create_directories("outputs");
            
            if (optimizer.exportResult(result, output_file)) {
                std::cout << "Results exported to: " << output_file << std::endl;
            } else {
                std::cout << "Warning: Could not export results" << std::endl;
            }
            
        } else {
            std::cerr << "Optimization failed: " << result.message << std::endl;
            return -1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Optimization completed successfully!" << std::endl;
    return 0;
}