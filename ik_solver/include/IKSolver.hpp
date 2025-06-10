#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP

#include <vector>
#include "../../robot_arm/include/Vector3D.hpp"

class RobotArm;

#define MAX_ITERATIONS 1000
#define TOLERANCE 0.0001f // in meters = 0.1mm

class IKSolver
{
public:
    float tolerance;   // Hoe dicht bij de doelpositie
    int maxIterations; // Maximale aantal iteraties
    RobotArm *arm;     // Verwijzing naar de robotarm

    IKSolver(RobotArm *arm, float tolerance = TOLERANCE, int maxIterations = MAX_ITERATIONS);
    // Geef positie mee en gewilde orientatie van de z-as van de end effector 
    std::vector<float> solveIK(const Vector3D &target, const Eigen::Matrix3f &R_des);


    
    // Los alleen de positie op (joints 1–3)
    void solvePositionOnly(const Vector3D &target, std::vector<float> &result);

    // Los alleen de oriëntatie op (joints 4–6)
    void solveOrientationOnly(const Eigen::Matrix3f &R_des, std::vector<float> &result);
;
    // void solveOrientationOnly(const Vector3D &target, std::vector<float> &result);

    Vector3D getEndEffector(const std::vector<float> &jointAngles) const;

};

#endif // IKSOLVER_HPP
