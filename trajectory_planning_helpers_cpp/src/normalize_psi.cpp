#include "trajectory_planning_helpers/trajectory_planning_helpers.hpp"
#include <cmath>

namespace trajectory_planning_helpers {

VectorXd normalize_psi(const VectorXd& psi) {
    VectorXd normalized(psi.size());
    
    for (int i = 0; i < psi.size(); ++i) {
        normalized(i) = normalize_psi(psi(i));
    }
    
    return normalized;
}

double normalize_psi(double psi) {
    while (psi > M_PI) {
        psi -= 2.0 * M_PI;
    }
    while (psi < -M_PI) {
        psi += 2.0 * M_PI;
    }
    return psi;
}

} // namespace trajectory_planning_helpers