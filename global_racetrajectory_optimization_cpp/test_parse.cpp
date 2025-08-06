#include "global_racetrajectory_optimization/global_racetrajectory_optimization.hpp"
#include <iostream>

int main() {
    auto config = global_racetrajectory_optimization::utils::parseConfigFile("params/racecar.ini");
    
    std::cout << "Found keys:" << std::endl;
    for (const auto& pair : config) {
        if (pair.first.find("stepsize") \!= std::string::npos) {
            std::cout << "Key: '" << pair.first << "' -> Value: '" << pair.second << "'" << std::endl;
        }
    }
    
    return 0;
}
EOF < /dev/null