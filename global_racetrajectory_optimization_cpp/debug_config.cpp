#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include <iostream>

int main() {
    try {
        auto config = global_racetrajectory_optimization::utils::parseConfigFile("params/racecar.ini");
        
        std::cout << "All parsed keys:" << std::endl;
        for (const auto& pair : config) {
            std::cout << "'" << pair.first << "' = '" << pair.second << "'" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    
    return 0;
}
