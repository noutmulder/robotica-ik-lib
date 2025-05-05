#ifndef INVERSEKINEMATICS_HPP
#define INVERSEKINEMATICS_HPP

#include "../../robot_arm/include/Joint.hpp"
#include "../../robot_arm/include/Link.hpp"
#include "IKSolver.hpp"
#include <cmath> 
#include <vector>

class InverseKinematics {
public:
    std::vector<Joint> joints;      // List of joints
    std::vector<Link> links;        // List of links
    IKSolver* ikSolver;             // Reference to the IK solver

    // Constructor to initialize the arm with joints and links
    InverseKinematics(std::vector<Joint> joints, std::vector<Link> links, IKSolver* solver);

    // Moves the end effector to a target position
    void moveTo(const Vector3D& target);

    // Rotates a specific joint by a given angle
    void rotateJoint(int index, float angle);

    // Gets the current position of the end effector
    Vector3D getEndEffector() const;

    // Sets the joints to the specified positions
    void setJoints(const std::vector<Joint>& newJoints);
};

#endif // INVERSEKINEMATICS_HPP
