#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP

class InverseKinematics;

#include <vector>
#include "../../robot_arm/include/Vector3D.hpp"
#include "IKSolver.hpp"
#include "InverseKinematics.hpp"


#define MAX_ITERATIONS 1000
#define TOLERANCE 0.01

class IKSolver {
public:
    InverseKinematics* arm;       // Verwijzing naar de arm (pointer naar InverseKinematics)
    float tolerance;              // Hoe dicht bij de doelpositie
    int maxIterations;            // Maximale aantal iteraties

    // Constructor voor IKSolver, initialiseert met arm, tolerantie en maximaal aantal iteraties
    IKSolver(InverseKinematics* arm, float tolerance = TOLERANCE, int maxIterations = MAX_ITERATIONS);

    // Lost de inverse kinematica op voor de gegeven doelpositie
    std::vector<float> solveIK(const Vector3D& target);

    // Bereken de huidige eind-effector positie (voor simpele kinematica)
    Vector3D getEndEffector(const std::vector<float>& jointAngles) const;

   
};


#endif // IKSOLVER_HPP
