#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include <iostream>
#include <fstream>

using namespace global_racetrajectory_optimization;

int main() {
    std::cout << "Basic Race Trajectory Optimization Example" << std::endl;
    
    try {
        // Create optimizer
        GlobalRaceTrajectoryOptimizer optimizer;
        
        // Create a simple test track (rectangular)
        MatrixXd test_track(4, 4);
        test_track << 0.0, 0.0, 5.0, 5.0,    // Point 1: x, y, w_right, w_left
                      100.0, 0.0, 5.0, 5.0,  // Point 2
                      100.0, 50.0, 5.0, 5.0, // Point 3
                      0.0, 50.0, 5.0, 5.0;   // Point 4
        
        // Save test track to file
        std::ofstream track_file("test_track.csv");
        track_file << "x_m,y_m,w_tr_right_m,w_tr_left_m" << std::endl;
        for (int i = 0; i < test_track.rows(); ++i) {
            track_file << test_track(i, 0) << "," << test_track(i, 1) << ","
                       << test_track(i, 2) << "," << test_track(i, 3) << std::endl;
        }
        track_file.close();
        
        // Load the test track
        if (!optimizer.loadTrack("test_track.csv")) {
            std::cerr << "Failed to load test track!" << std::endl;
            return -1;
        }
        
        // Prepare track
        if (!optimizer.prepareTrack(true)) {
            std::cerr << "Failed to prepare track!" << std::endl;
            return -1;
        }
        
        // Run shortest path optimization
        std::cout << "\nRunning shortest path optimization..." << std::endl;
        auto result = optimizer.optimizeShortestPath();
        
        if (result.success) {
            optimizer.visualizeResult(result);
            
            // Export results
            if (optimizer.exportResult(result, "test_optimization_result.csv")) {
                std::cout << "Results exported to test_optimization_result.csv" << std::endl;
            }
        } else {
            std::cerr << "Optimization failed: " << result.message << std::endl;
            return -1;
        }
        
        // Try minimum curvature optimization
        std::cout << "\nRunning minimum curvature optimization..." << std::endl;
        auto result2 = optimizer.optimizeMinCurvature(false);
        
        if (result2.success) {
            std::cout << "\n--- Minimum Curvature Results ---" << std::endl;
            optimizer.visualizeResult(result2);
        } else {
            std::cerr << "Min curvature optimization failed: " << result2.message << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "\nExample completed successfully!" << std::endl;
    return 0;
}