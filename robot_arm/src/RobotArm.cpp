#include "RobotArm.hpp"
#include "Joint.hpp"
#include "Vector3D.hpp"

// Constructor: Initialiseer de gewrichten, inverse kinematica en solver
RobotArm::RobotArm() {
    joints = std::vector<Joint>{
        Joint(0.0f, JOINT1_MIN_ANGLE, JOINT1_MAX_ANGLE, new Link(LINK1_LENGTH, 1.0f, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT2_MIN_ANGLE, JOINT2_MAX_ANGLE, new Link(LINK2_LENGTH, 1.0f, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT3_MIN_ANGLE, JOINT3_MAX_ANGLE, new Link(LINK3_LENGTH, 1.0f, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT4_MIN_ANGLE, JOINT4_MAX_ANGLE, new Link(LINK4_LENGTH, 1.0f, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT5_MIN_ANGLE, JOINT5_MAX_ANGLE, new Link(LINK5_LENGTH, 1.0f, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT6_MIN_ANGLE, JOINT6_MAX_ANGLE, new Link(LINK6_LENGTH, 1.0f, Vector3D(0, 0, 0))),
    };

    ikSolver = new IKSolver(this);
    ik = new InverseKinematics(*this, ikSolver);
}

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
    if (jointIndex >= 0 && jointIndex < static_cast<int>(joints.size())) {
        joints[jointIndex].setAngle(angle);
        ik->setJoints(joints); // Update de gewrichten in de inverse kinematica
    } else {
        // Error-handling voor ongeldige jointIndex
        std::cerr << "Invalid joint index!" << std::endl;
    }
}

Vector3D RobotArm::getEndEffectorPosition() {
    // Collect all joint angles
    std::vector<float> jointAngles;
    for (const auto& joint : joints) {
        jointAngles.push_back(joint.angle);  // Collect each joint's current angle
    }

    // Use the InverseKinematics to get the position of the end effector
    return ik->getEndEffector(jointAngles);  
}
