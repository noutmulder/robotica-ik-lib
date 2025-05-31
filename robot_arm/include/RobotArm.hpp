#ifndef ROBOTARM_HPP
#define ROBOTARM_HPP

#include "../../ik_solver/include/InverseKinematics.hpp"
#include "../../ik_solver/include/IKSolver.hpp"
#include "Joint.hpp"
#include <vector>
#include <iostream>
#include "ARM_CONFIG.hpp"
#include <Eigen/Dense>

using namespace Eigen;

class InverseKinematics;
class IKSolver;
class Vector3D;

class RobotArm
{
public:
    std::vector<Joint> joints; // Lijst van gewrichten
    InverseKinematics *ik;     // Pointer naar inverse kinematica
    IKSolver *ikSolver;        // Pointer naar inverse kinematica solver

    // Constructor
    RobotArm();

    // Beweeg de arm naar een doelpositie
    void moveTo(Vector3D target);

    // Draai een gewricht naar een specifieke hoek
    void rotateJoint(int jointIndex, float angle);

    // Functie om de end-effector positie te krijgen
    Vector3D getEndEffectorPosition();
};

Matrix4f createTransformFromRPYAndTranslation(const Vector3D &rpy, const Vector3D &translation);

#endif // ROBOTARM_HPP
