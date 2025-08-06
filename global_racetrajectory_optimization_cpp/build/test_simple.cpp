#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include <iostream>

using namespace global_racetrajectory_optimization;

int main() {
    std::cout << "Simple Test Program" << std::endl;
    
    try {
        // Create a very simple track
        MatrixXd simple_track(4, 4);
        simple_track << 0.0, 0.0, 5.0, 5.0,
                        20.0, 0.0, 5.0, 5.0,
                        20.0, 20.0, 5.0, 5.0,
                        0.0, 20.0, 5.0, 5.0;
        
        std::cout << "Created simple rectangular track" << std::endl;
        
        // Test track validity
        bool valid = utils::checkTrackValidity(simple_track);
        std::cout << "Track valid: " << (valid ? "YES" : "NO") << std::endl;
        
        // Calculate some basic properties
        std::cout << "Track points: " << simple_track.rows() << std::endl;
        std::cout << "Track columns: " << simple_track.cols() << std::endl;
        
        // Test shortest path (which should just use centerline)
        VectorXd alpha = VectorXd::Zero(simple_track.rows());
        MatrixXd normvectors = MatrixXd::Zero(simple_track.rows(), 2);
        
        // Create simple normal vectors (perpendicular to segments)
        for (int i = 0; i < simple_track.rows(); ++i) {
            int next_i = (i + 1) % simple_track.rows();
            Vector2d direction = simple_track.row(next_i).head(2) - simple_track.row(i).head(2);
            direction.normalize();
            // Perpendicular vector (90 degrees)
            normvectors.row(i) << -direction(1), direction(0);
        }
        
        // Calculate raceline
        auto raceline = utils::calculateRaceline(simple_track, normvectors, alpha);
        std::cout << "Raceline calculated: " << raceline.rows() << " points" << std::endl;
        
        // Calculate element lengths
        VectorXd el_lengths(simple_track.rows());  // Note: using same size for closed track
        for (int i = 0; i < simple_track.rows(); ++i) {
            int next_i = (i + 1) % simple_track.rows();
            el_lengths(i) = (simple_track.row(next_i).head(2) - simple_track.row(i).head(2)).norm();
        }
        
        std::cout << "Element lengths: " << el_lengths.transpose() << std::endl;
        
        // Test curvature calculation
        auto curvature = utils::calculateCurvature(raceline, el_lengths.head(el_lengths.size()-1), true);
        std::cout << "Curvature calculated successfully" << std::endl;
        
        // Test velocity calculation
        VectorXd velocity = VectorXd::Constant(raceline.rows(), 15.0);
        double lap_time = utils::calculateLapTime(velocity, el_lengths.head(el_lengths.size()-1));
        std::cout << "Lap time: " << lap_time << " s" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Simple test completed successfully!" << std::endl;
    return 0;
}