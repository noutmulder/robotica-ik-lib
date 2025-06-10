#ifndef INVERSEKINEMATICS_HPP
#define INVERSEKINEMATICS_HPP

#include "../../robot_arm/include/RobotArm.hpp"
#include "IKSolver.hpp"
#include <cmath>
#include <vector>

class IKSolver;
class RobotArm;
class Joint;

class InverseKinematics
{
public:
    RobotArm &robotArm; // Referentie naar de RobotArm
    IKSolver *ikSolver; // Reference to the IK solver

    // Constructor om de arm met de RobotArm en IK solver te initialiseren
    InverseKinematics(RobotArm &arm, IKSolver *solver);

    // Moves the end effector to a target position
    void moveTo(const Vector3D &target, const Eigen::Matrix3f &R_des);

    // Rotates a specific joint by a given angle
    void rotateJoint(int index, float angle);

    // Sets the joints to the specified positions
    void setJoints(const std::vector<Joint> &newJoints);

    // Sets the joint angles to the specified values
    bool setJointAngles(const std::vector<float> &angles);

    // Functie om de gewrichtshoeken te krijgen
    std::vector<float> getJointAngles() const;
};

#endif // INVERSEKINEMATICS_HPP
