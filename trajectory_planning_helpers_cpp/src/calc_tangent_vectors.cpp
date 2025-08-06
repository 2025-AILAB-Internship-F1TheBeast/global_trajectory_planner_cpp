#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>

namespace trajectory_planning_helpers {

MatrixXd calc_tangent_vectors(const VectorXd& psi) {
    int n = psi.size();
    MatrixXd tangent_vectors(2, n);
    
    for (int i = 0; i < n; ++i) {
        tangent_vectors(0, i) = std::cos(psi(i));  // x component  
        tangent_vectors(1, i) = std::sin(psi(i));  // y component
    }
    
    return tangent_vectors;
}

} // namespace trajectory_planning_helpers