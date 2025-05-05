#include "RobotArm.hpp"
#include "Joint.hpp"
#include "Vector3D.hpp"

// Constructor: Initialiseer de gewrichten, inverse kinematica en solver
RobotArm::RobotArm(std::vector<Joint> joints, InverseKinematics* ik, IKSolver* ikSolver)
    : joints(joints), ik(ik), ikSolver(ikSolver) {}

// Beweeg de arm naar een doelpositie
void RobotArm::moveTo(Vector3D target) {
    // Gebruik de inverse kinematica solver om de hoeken voor de gewrichten te berekenen
    std::vector<float> angles = ikSolver->solveIK(target);

    // Pas de hoeken van de gewrichten aan
    for (size_t i = 0; i < angles.size(); ++i) {
        joints[i].setAngle(angles[i]);
    }

    // Zet de gewrichten in de inverse kinematica manager
    ik->setJoints(joints);
}

// Draai een gewricht naar een specifieke hoek
void RobotArm::rotateJoint(int jointIndex, float angle) {
    // Controleer of de jointIndex geldig is
    if (jointIndex >= 0 && jointIndex < joints.size()) {
        joints[jointIndex].setAngle(angle);
        ik->setJoints(joints); // Update de gewrichten in de inverse kinematica
    } else {
        // Error-handling voor ongeldige jointIndex
        std::cerr << "Invalid joint index!" << std::endl;
    }
}
