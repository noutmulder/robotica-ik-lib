#include "InverseKinematics.hpp"
#include <iostream>
#include "IKSolver.hpp"

// Constructor: koppelt de inverse kinematica aan de robotarm en de solver
InverseKinematics::InverseKinematics(RobotArm &arm, IKSolver *solver)
    : robotArm(arm), ikSolver(solver) {}

// Beweeg de grijper naar een gewenste positie én oriëntatie
void InverseKinematics::moveTo(const Vector3D &target, const Eigen::Matrix3f &R_des)
{
    // Vraag inverse kinematica oplossing op
    std::vector<float> angles = ikSolver->solveIK(target, R_des);

    // Zet de berekende hoeken in de robotarm
    for (size_t i = 0; i < robotArm.joints.size(); ++i)
    {
        robotArm.joints[i].setAngle(angles[i]);
    }
}

// Roteer één gewricht naar een specifieke hoek (in graden)
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

// Zet alle gewrichten in één keer (bijv. vanuit andere armstructuur)
void InverseKinematics::setJoints(const std::vector<Joint> &newJoints)
{
    robotArm.joints = newJoints;
}

// Zet gewrichtshoeken met grenscontrole (clamping binnen min/max)
bool InverseKinematics::setJointAngles(const std::vector<float> &angles)
{
    size_t n = std::min(angles.size(), robotArm.joints.size());

    for (size_t i = 0; i < n; ++i)
    {
        float minAngle = robotArm.joints[i].minAngle;
        float maxAngle = robotArm.joints[i].maxAngle;
        float target = angles[i];

        // Clamp naar toegestane grenzen
        if (target < minAngle)
        {
            std::cout << "Joint[" << i << "] below minimum! (" << target << "° → " << minAngle << "°)\n";
            target = minAngle;
        }
        else if (target > maxAngle)
        {
            std::cout << "Joint[" << i << "] above maximum! (" << target << "° → " << maxAngle << "°)\n";
            target = maxAngle;
        }

        // Alleen aanpassen als de hoek echt verandert
        float before = robotArm.joints[i].getAngle();
        robotArm.joints[i].setAngle(target);
        float after = robotArm.joints[i].getAngle();

        if (std::abs(before - after) > 1e-5f)
        {
            std::cout << "Joint[" << i << "] updated from " << before << "° to " << after << "°\n";
        }
        else
        {
            std::cout << "Joint[" << i << "] remains unchanged at " << after << "°\n";
        }
    }

    return true;
}

// Haal de huidige hoeken van alle gewrichten op
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
