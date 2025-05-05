#include "RobotArm.hpp"
#include "Joint.hpp"
#include "Link.hpp"
#include "Vector3D.hpp"
#include "IKSolver.hpp"
#include <iostream>
#include <vector>

int main() {
    // Create Vector3D objects for joint positions (set the positions arbitrarily)
    Vector3D jointPos1(0.0, 0.0, 0.0);  // Origin
    Vector3D jointPos2(1.0, 0.0, 0.0);  // 1 unit along x-axis
    Vector3D jointPos3(2.0, 0.0, 0.0);  // 1 unit further along x-axis
    Vector3D jointPos4(3.0, 0.0, 0.0);  // Another link along x-axis
    Vector3D jointPos5(4.0, 0.0, 0.0);  // Another link along x-axis
    Vector3D jointPos6(5.0, 0.0, 0.0);  // Final link along x-axis

    // Create Link objects (arbitrary length and mass values)
    Link link1(1.0, 1.0, jointPos1);
    Link link2(1.0, 1.0, jointPos2);
    Link link3(1.0, 1.0, jointPos3);
    Link link4(1.0, 1.0, jointPos4);
    Link link5(1.0, 1.0, jointPos5);
    Link link6(1.0, 1.0, jointPos6);

    // Create Joint objects with initial angles (set to 0 for simplicity)
    Joint joint1(0.0, -45.0, 45.0, &link1);  // Initial angle 0, range -45 to 45 degrees
    Joint joint2(0.0, -45.0, 45.0, &link2);
    Joint joint3(0.0, -45.0, 45.0, &link3);
    Joint joint4(0.0, -45.0, 45.0, &link4);
    Joint joint5(0.0, -45.0, 45.0, &link5);
    Joint joint6(0.0, -45.0, 45.0, &link6);

    // Create a vector of joints
    std::vector<Joint> joints = {joint1, joint2, joint3, joint4, joint5, joint6};

    // Create a mock IK solver (simplified, no inverse kinematics calculation)
    IKSolver* ikSolver = new IKSolver(nullptr, 0.01, 1000);  // Just initialize with a dummy IK instance

    // Create the RobotArm object
    RobotArm robotArm(joints, nullptr, ikSolver);  // No need for inverse kinematics for this test

    // Test: Get the current end effector position (without any movements)
    std::cout << "Getting current end effector position...\n";
    Vector3D endEffectorPosition = robotArm.getEndEffectorPosition();
    
    // Print the result
    std::cout << "Current end effector position: (" 
              << endEffectorPosition.x << ", "
              << endEffectorPosition.y << ", "
              << endEffectorPosition.z << ")\n";

    // Cleanup
    delete ikSolver;

    return 0;
}
