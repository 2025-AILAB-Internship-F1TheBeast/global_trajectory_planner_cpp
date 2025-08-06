#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include <iostream>
#include <fstream>

using namespace global_racetrajectory_optimization;

int main() {
    std::cout << "Track Analysis Example" << std::endl;
    
    try {
        // Test utility functions
        
        // 1. Test track import
        std::cout << "\n1. Testing track import..." << std::endl;
        
        // Create a test CSV track file
        std::ofstream test_file("analysis_test_track.csv");
        test_file << "x_m,y_m,w_tr_right_m,w_tr_left_m" << std::endl;
        test_file << "0.0,0.0,4.0,4.0" << std::endl;
        test_file << "20.0,10.0,4.5,3.5" << std::endl;
        test_file << "30.0,30.0,3.0,5.0" << std::endl;
        test_file << "10.0,40.0,4.0,4.0" << std::endl;
        test_file << "-10.0,20.0,4.0,4.0" << std::endl;
        test_file.close();
        
        auto track = utils::importTrack("analysis_test_track.csv");
        std::cout << "Track imported with " << track.rows() << " points" << std::endl;
        
        // 2. Test track validity
        std::cout << "\n2. Testing track validity..." << std::endl;
        bool valid = utils::checkTrackValidity(track);
        std::cout << "Track validity: " << (valid ? "PASS" : "FAIL") << std::endl;
        
        // 3. Test new start point setting
        std::cout << "\n3. Testing new start point..." << std::endl;
        Vector2d new_start(15.0, 25.0);
        auto reordered_track = utils::setNewStartPoint(track, new_start);
        std::cout << "Track reordered with new start point" << std::endl;
        
        // 4. Test raceline calculation
        std::cout << "\n4. Testing raceline calculation..." << std::endl;
        
        // Create some dummy normal vectors and alpha values
        MatrixXd normvectors(track.rows(), 2);
        VectorXd alpha = VectorXd::Random(track.rows()) * 2.0; // Random alpha values
        
        for (int i = 0; i < track.rows(); ++i) {
            // Simple perpendicular normal vectors (not accurate, just for testing)
            Vector2d direction;
            if (i < track.rows() - 1) {
                direction = track.row(i + 1).head(2) - track.row(i).head(2);
            } else {
                direction = track.row(0).head(2) - track.row(i).head(2);
            }
            direction.normalize();
            
            // Perpendicular vector
            normvectors.row(i) << -direction(1), direction(0);
        }
        
        auto raceline = utils::calculateRaceline(track, normvectors, alpha);
        std::cout << "Raceline calculated with " << raceline.rows() << " points" << std::endl;
        
        // 5. Test curvature calculation
        std::cout << "\n5. Testing curvature calculation..." << std::endl;
        
        VectorXd el_lengths(track.rows() - 1);
        for (int i = 0; i < el_lengths.size(); ++i) {
            el_lengths(i) = (track.row(i + 1).head(2) - track.row(i).head(2)).norm();
        }
        
        auto curvature = utils::calculateCurvature(raceline, el_lengths, false);
        std::cout << "Curvature calculated:" << std::endl;
        std::cout << "  Min curvature: " << curvature.minCoeff() << " rad/m" << std::endl;
        std::cout << "  Max curvature: " << curvature.maxCoeff() << " rad/m" << std::endl;
        std::cout << "  Mean curvature: " << curvature.mean() << " rad/m" << std::endl;
        
        // 6. Test lap time calculation
        std::cout << "\n6. Testing lap time calculation..." << std::endl;
        
        VectorXd velocity = VectorXd::Constant(raceline.rows(), 25.0); // 25 m/s constant
        double lap_time = utils::calculateLapTime(velocity, el_lengths);
        std::cout << "Lap time (25 m/s constant): " << lap_time << " s" << std::endl;
        
        // 7. Test export functionality
        std::cout << "\n7. Testing export functionality..." << std::endl;
        
        OptimizationResult dummy_result;
        dummy_result.success = true;
        dummy_result.raceline = raceline;
        dummy_result.kappa_opt = curvature;
        dummy_result.v_opt = velocity;
        dummy_result.s_opt = VectorXd::LinSpaced(raceline.rows(), 0.0, el_lengths.sum());
        dummy_result.alpha_opt = alpha;
        dummy_result.lap_time = lap_time;
        dummy_result.optimization_time = 0.1;
        dummy_result.message = "Test result";
        
        bool export_success = utils::exportToCSV(dummy_result, "analysis_test_result.csv");
        std::cout << "Export to CSV: " << (export_success ? "SUCCESS" : "FAILED") << std::endl;
        
        // Clean up test files
        std::remove("analysis_test_track.csv");
        std::remove("analysis_test_result.csv");
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "\nAll track analysis tests completed!" << std::endl;
    return 0;
}