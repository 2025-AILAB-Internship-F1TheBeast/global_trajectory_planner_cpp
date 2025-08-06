#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>

namespace trajectory_planning_helpers {

MatrixXd calc_normal_vectors(const VectorXd& psi) {
    int n = psi.size();
    MatrixXd normal_vectors(2, n);
    
    for (int i = 0; i < n; ++i) {
        normal_vectors(0, i) = std::cos(psi(i) + M_PI / 2.0);  // x component
        normal_vectors(1, i) = std::sin(psi(i) + M_PI / 2.0);  // y component
    }
    
    return normal_vectors;
}

} // namespace trajectory_planning_helpers