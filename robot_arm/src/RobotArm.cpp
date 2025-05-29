#include "RobotArm.hpp"
#include "Joint.hpp"
#include "Vector3D.hpp"
#include "RobotArm.hpp"
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;


// Constructor: Initialiseer de gewrichten, inverse kinematica en solver
RobotArm::RobotArm()
{
    joints = std::vector<Joint>{
        Joint(0.0f, JOINT1_MIN_ANGLE, JOINT1_MAX_ANGLE,
              new Link(LINK1_LENGTH, LINK1_WEIGHT),
              JOINT1_ORIGIN, JOINT1_RPY, JOINT1_AXIS),

        Joint(0.0f, JOINT2_MIN_ANGLE, JOINT2_MAX_ANGLE,
              new Link(LINK2_LENGTH, LINK2_WEIGHT),
              JOINT2_ORIGIN, JOINT2_RPY, JOINT2_AXIS),

        Joint(0.0f, JOINT3_MIN_ANGLE, JOINT3_MAX_ANGLE,
              new Link(LINK3_LENGTH, LINK3_WEIGHT),
              JOINT3_ORIGIN, JOINT3_RPY, JOINT3_AXIS),

        Joint(0.0f, JOINT4_MIN_ANGLE, JOINT4_MAX_ANGLE,
              new Link(LINK4_LENGTH, LINK4_WEIGHT),
              JOINT4_ORIGIN, JOINT4_RPY, JOINT4_AXIS),

        Joint(0.0f, JOINT5_MIN_ANGLE, JOINT5_MAX_ANGLE,
              new Link(LINK5_LENGTH, LINK5_WEIGHT),
              JOINT5_ORIGIN, JOINT5_RPY, JOINT5_AXIS),

        Joint(0.0f, JOINT6_MIN_ANGLE, JOINT6_MAX_ANGLE,
              new Link(LINK6_LENGTH, LINK6_WEIGHT),
              JOINT6_ORIGIN, JOINT6_RPY, JOINT6_AXIS),
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


Vector3D RobotArm::getEndEffectorPosition() {
    // Begin met de identiteitsmatrix (geen rotatie/translatie)
    Matrix4f currentTransform = Matrix4f::Identity();

    for (const Joint& joint : joints) {
        // 1. Basistransformatie uit URDF (origin + rpy)
        Matrix4f baseTransform = createTransformFromRPYAndTranslation(joint.rpy, joint.origin);

        // 2. Extra rotatie om de joint-as (de eigenlijke joint-rotatie)
        float angleRad = joint.getAngle() * (M_PI / 180.0f);
        Vector3f axis = joint.axis.toEigen().normalized();
        Matrix3f jointRotMatrix = AngleAxisf(angleRad, axis).toRotationMatrix();

        // Zet om naar een 4x4 matrix
        Matrix4f jointRotation = Matrix4f::Identity();
        jointRotation.block<3,3>(0,0) = jointRotMatrix;

        // 3. Combineer: transform * base * rotatie
        currentTransform = currentTransform * baseTransform * jointRotation;
    }

    // Extract de positie van de end-effector uit de matrix (laatste kolom)
    Vector3f endPos = currentTransform.block<3,1>(0,3);
    return Vector3D(endPos.x(), endPos.y(), endPos.z());
}


// Maakt een 4x4 transformatiematrix uit RPY (in radialen) + translatie
Matrix4f createTransformFromRPYAndTranslation(const Vector3D& rpy, const Vector3D& translation) {
    // Bereken rotatiematrix vanuit RPY (in radialen)
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(rpy.z, Eigen::Vector3f::UnitZ()) *
               Eigen::AngleAxisf(rpy.y, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(rpy.x, Eigen::Vector3f::UnitX());

    // Bouw de volledige 4x4 matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation.toEigen();

    return transform;
}


