#include "InverseKinematics.hpp"
#include <iostream>
#include "IKSolver.hpp"

// Constructor om de arm met de RobotArm en IK solver te initialiseren
InverseKinematics::InverseKinematics(RobotArm& arm, IKSolver* solver)
    : robotArm(arm), ikSolver(solver) {}

void InverseKinematics::moveTo(const Vector3D& target) {
    std::vector<float> angles = ikSolver->solveIK(target);  // Roep de solveIK functie aan van de solver

    for (size_t i = 0; i < robotArm.joints.size(); ++i) {
        robotArm.joints[i].setAngle(angles[i]);
    }
}

void InverseKinematics::rotateJoint(int index, float angle) {
    if (index >= 0 && index < robotArm.joints.size()) {
        robotArm.joints[index].setAngle(angle);
    } else {
        std::cout << "Invalid joint index!" << std::endl;
    }
}

Vector3D InverseKinematics::getEndEffector(const std::vector<float>& jointAngles) const {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float currentAngle = 0.0f;

    for (size_t i = 0; i < jointAngles.size(); ++i) {
        Link* currentLink = robotArm.joints[i].link;
        float length = currentLink->length;

        currentAngle += jointAngles[i];

        x += length * cos(currentAngle);
        y += length * sin(currentAngle);
    }

    return Vector3D(x, y, z);
}

void InverseKinematics::setJoints(const std::vector<Joint>& newJoints) {
    robotArm.joints = newJoints;
}
