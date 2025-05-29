#include "InverseKinematics.hpp"
#include <iostream>
#include "IKSolver.hpp"

// Constructor om de arm met de RobotArm en IK solver te initialiseren
InverseKinematics::InverseKinematics(RobotArm &arm, IKSolver *solver)
    : robotArm(arm), ikSolver(solver) {}

void InverseKinematics::moveTo(const Vector3D &target)
{
    std::vector<float> angles = ikSolver->solveIK(target); // Roep de solveIK functie aan van de solver

    for (size_t i = 0; i < robotArm.joints.size(); ++i)
    {
        robotArm.joints[i].setAngle(angles[i]);
    }
}

void InverseKinematics::rotateJoint(int index, float angle)
{
    if (index >= 0 && index < robotArm.joints.size())
    {
        robotArm.joints[index].setAngle(angle);
    }
    else
    {
        std::cout << "Invalid joint index!" << std::endl;
    }
}

Vector3D InverseKinematics::getEndEffector(const std::vector<float>& jointAngles) const {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float theta = 0.0f; // Accumulated rotation in radians

    for (size_t i = 0; i < jointAngles.size(); ++i) {
        float angleRad = jointAngles[i] * (M_PI / 180.0f); // Degrees to radians
        theta += angleRad;

        float length = robotArm.joints[i].link->length;
        x += length * cos(theta);
        y += length * sin(theta);
    }

    return Vector3D(x, y, z);
}


void InverseKinematics::setJoints(const std::vector<Joint> &newJoints)
{
    robotArm.joints = newJoints;
}

bool InverseKinematics::setJointAngles(const std::vector<float> &angles)
{
    size_t n = std::min(angles.size(), robotArm.joints.size());

    for (size_t i = 0; i < n; ++i)
    {
        float minAngle = robotArm.joints[i].minAngle;
        float maxAngle = robotArm.joints[i].maxAngle;
        float target = angles[i];

        if (target < minAngle) {
            std::cout << "Joint[" << i << "] below minimum! (" << target << "° → " << minAngle << "°)\n";
            target = minAngle;
        } else if (target > maxAngle) {
            std::cout << "Joint[" << i << "] above maximum! (" << target << "° → " << maxAngle << "°)\n";
            target = maxAngle;
        }

        float before = robotArm.joints[i].getAngle();
        robotArm.joints[i].setAngle(target);
        float after = robotArm.joints[i].getAngle();

        if (std::abs(before - after) > 1e-5f) {
            std::cout << "Joint[" << i << "] updated from " << before << "° to " << after << "°\n";
        } else {
            std::cout << "Joint[" << i << "] remains unchanged at " << after << "°\n";
        }
    }

    return true;
}





std::vector<float> InverseKinematics::getJointAngles() const
{
    std::vector<float> angles;
    angles.reserve(robotArm.joints.size());

    for (const auto &joint : robotArm.joints)
    {
        angles.push_back(joint.getAngle());
    }

    return angles;
}
