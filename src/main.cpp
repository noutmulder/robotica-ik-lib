#include "RobotArm.hpp"
#include "Joint.hpp"
#include "Link.hpp"
#include "Vector3D.hpp"
#include "InverseKinematics.hpp"
#include "IKSolver.hpp"
#include <iostream>
#include <vector>

int main() {
    // Create Vector3D objects for positions
    Vector3D jointPos1(0.0, 0.0, 0.0);
    Vector3D jointPos2(1.0, 0.0, 0.0);
    Vector3D jointPos3(2.0, 0.0, 0.0);

    // Create Link objects (for simplicity, we will assume the mass and length are arbitrary)
    Link link1(1.0, 1.0, jointPos1);
    Link link2(1.0, 1.0, jointPos2);
    Link link3(1.0, 1.0, jointPos3);

    // Create Joint objects
    Joint joint1(0.0, -45.0, 45.0, &link1);  // Initial angle is 0, range -45 to 45 degrees
    Joint joint2(0.0, -45.0, 45.0, &link2);
    Joint joint3(0.0, -45.0, 45.0, &link3);

    // Create a vector of joints
    std::vector<Joint> joints = {joint1, joint2, joint3};

    // Create a mock IK solver and inverse kinematics object (simplified for the test)
    InverseKinematics* ik = new InverseKinematics(joints, {link1, link2, link3}, nullptr);  // Assuming the InverseKinematics constructor works like this
    IKSolver* ikSolver = new IKSolver(ik, 0.01, 1000);

    // Create the RobotArm object
    RobotArm robotArm(joints, ik, ikSolver);

    // Test: Move the arm to a new target position (e.g., Vector3D(2.0, 2.0, 0.0))
    Vector3D targetPosition(2.0, 2.0, 0.0);
    std::cout << "Moving robot arm to target position (2.0, 2.0, 0.0)...\n";
    robotArm.moveTo(targetPosition);

    // Test: Rotate a specific joint (rotate joint 1 to 30 degrees)
    std::cout << "Rotating joint 1 to 30 degrees...\n";
    robotArm.rotateJoint(0, 30.0);  // Rotate the first joint to 30 degrees

    // Test: Rotate another joint (rotate joint 2 to -30 degrees)
    std::cout << "Rotating joint 2 to -30 degrees...\n";
    robotArm.rotateJoint(1, -30.0);  // Rotate the second joint to -30 degrees

    // Cleanup
    delete ik;
    delete ikSolver;

    return 0;
}
