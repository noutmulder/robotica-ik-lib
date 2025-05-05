#ifndef ROBOTARM_HPP
#define ROBOTARM_HPP

#include "../../ik_solver/include/InverseKinematics.hpp"
#include "../../ik_solver/include/IKSolver.hpp"
#include "Joint.hpp"
#include <vector>
#include <iostream>

class RobotArm {
public:
    std::vector<Joint> joints;             // Lijst van gewrichten
    InverseKinematics* ik;                 // Pointer naar inverse kinematica (het object dat de gewrichten beheert)
    IKSolver* ikSolver;                    // Pointer naar de IK Solver die de berekeningen doet

    // Constructor
    RobotArm(std::vector<Joint> joints, InverseKinematics* ik, IKSolver* ikSolver);

    // Beweeg de arm naar een doelpositie
    void moveTo(Vector3D target);

    // Draai een gewricht naar een specifieke hoek
    void rotateJoint(int jointIndex, float angle);
};

#endif // ROBOTARM_HPP
