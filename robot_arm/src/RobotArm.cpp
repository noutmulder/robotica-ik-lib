#include "RobotArm.hpp"
#include "Joint.hpp"
#include "Vector3D.hpp"

// Constructor: Initialiseer de gewrichten, inverse kinematica en solver
RobotArm::RobotArm()
{
    joints = std::vector<Joint>{
        Joint(0.0f, JOINT1_MIN_ANGLE, JOINT1_MAX_ANGLE, new Link(LINK1_LENGTH, LINK1_WEIGHT, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT2_MIN_ANGLE, JOINT2_MAX_ANGLE, new Link(LINK2_LENGTH, LINK2_WEIGHT, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT3_MIN_ANGLE, JOINT3_MAX_ANGLE, new Link(LINK3_LENGTH, LINK3_WEIGHT, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT4_MIN_ANGLE, JOINT4_MAX_ANGLE, new Link(LINK4_LENGTH, LINK4_WEIGHT, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT5_MIN_ANGLE, JOINT5_MAX_ANGLE, new Link(LINK5_LENGTH, LINK5_WEIGHT, Vector3D(0, 0, 0))),
        Joint(0.0f, JOINT6_MIN_ANGLE, JOINT6_MAX_ANGLE, new Link(LINK6_LENGTH, LINK6_WEIGHT, Vector3D(0, 0, 0))),
    };

    ikSolver = new IKSolver(this);
    ik = new InverseKinematics(*this, ikSolver);
}

// Beweeg de arm naar een doelpositie
void RobotArm::moveTo(Vector3D target)
{
    // Gebruik de inverse kinematica solver om de hoeken voor de gewrichten te berekenen
    std::vector<float> angles = ikSolver->solveIK(target);

    // Pas de hoeken van de gewrichten aan
    for (size_t i = 0; i < angles.size(); ++i)
    {
        joints[i].setAngle(angles[i]);
    }

    // Zet de gewrichten in de inverse kinematica manager
    ik->setJoints(joints);
}

// Draai een gewricht naar een specifieke hoek
void RobotArm::rotateJoint(int jointIndex, float angle)
{
    // Controleer of de jointIndex geldig is
    if (jointIndex >= 0 && jointIndex < static_cast<int>(joints.size()))
    {
        joints[jointIndex].setAngle(angle);
        ik->setJoints(joints); // Update de gewrichten in de inverse kinematica
    }
    else
    {
        // Error-handling voor ongeldige jointIndex
        std::cerr << "Invalid joint index!" << std::endl;
    }
}

Vector3D RobotArm::getEndEffectorPosition()
{
    // Verzamel alle gewrichtshoeken
    std::vector<float> jointAngles; // Maak een lege vector om de gewrichtshoeken op te slaan
    for (const auto &joint : joints)
    {
        // Voeg de hoek van elk gewricht toe aan de vector
        jointAngles.push_back(joint.angle);
    }

    // Gebruik de InverseKinematica om de positie van de end effector te krijgen
    return ik->getEndEffector(jointAngles);
}
