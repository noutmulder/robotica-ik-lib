#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP

#include <vector>
#include "../../robot_arm/include/Vector3D.hpp"

class RobotArm;

#define MAX_ITERATIONS 1000
#define TOLERANCE 0.01

class IKSolver {
public:         
    float tolerance;              // Hoe dicht bij de doelpositie
    int maxIterations;           // Maximale aantal iteraties
    RobotArm* arm;               // Verwijzing naar de robotarm

    IKSolver(RobotArm* arm, float tolerance = TOLERANCE, int maxIterations = MAX_ITERATIONS);
    std::vector<float> solveIK(const Vector3D& target);
    Vector3D getEndEffector(const std::vector<float>& jointAngles) const;
};

#endif // IKSOLVER_HPP
