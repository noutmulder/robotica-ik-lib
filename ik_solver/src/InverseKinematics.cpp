#include "InverseKinematics.hpp"
#include <iostream>

// Constructor to initialize joints, links, and IK solver
InverseKinematics::InverseKinematics(std::vector<Joint> joints, std::vector<Link> links, IKSolver* solver)
    : joints(joints), links(links), ikSolver(solver) {}

// Moves the end effector to the target position using IK solver
void InverseKinematics::moveTo(const Vector3D& target) {
    if (ikSolver->isReachable(target)) {
        // Solve for the joint angles to reach the target
        std::vector<float> angles = ikSolver->solveIK(target);

        // Apply the calculated angles to the joints
        for (size_t i = 0; i < joints.size(); ++i) {
            joints[i].setAngle(angles[i]);
        }
    } else {
        std::cout << "Target is unreachable!" << std::endl;
    }
}

// Rotates a specific joint by a given angle
void InverseKinematics::rotateJoint(int index, float angle) {
    if (index >= 0 && index < joints.size()) {
        joints[index].setAngle(angle);
    } else {
        std::cout << "Invalid joint index!" << std::endl;
    }
}

// Gets the current position of the end effector by calculating the position from all joints and links
Vector3D InverseKinematics::getEndEffector() const {
    Vector3D endEffectorPos(0, 0, 0);

    // Loop through all joints and links to calculate the end effector position
    for (size_t i = 0; i < joints.size(); ++i) {
        // Apply the transformation based on joint angles and link lengths
        // For simplicity, we use a simple forward kinematics model
        float x = links[i].length * cos(joints[i].angle);
        float y = links[i].length * sin(joints[i].angle);
        endEffectorPos.addVector(Vector3D(x, y, 0));  // Simplified 2D movement for now
    }

    return endEffectorPos;
}

// Sets the joints to new positions
void InverseKinematics::setJoints(const std::vector<Joint>& newJoints) {
    joints = newJoints;
}
