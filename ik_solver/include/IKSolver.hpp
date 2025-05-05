#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP

class InverseKinematics;

#include <vector>

#define MAX_ITERATIONS 1000
#define TOLERANCE 0.01

class IKSolver {
public:
    InverseKinematics* arm;       // Reference to the arm (pointer to InverseKinematics)
    float tolerance;              // How close to the target position
    int maxIterations;            // Maximum number of iterations

    // Constructor for IKSolver, initialize with arm, tolerance, and max iterations
    IKSolver(InverseKinematics* arm, float tolerance = TOLERANCE, int maxIterations = MAX_ITERATIONS);

    // Solves the inverse kinematics for the given target position
    std::vector<float> solveIK(const Vector3D& target);

    // Checks if the target is reachable based on the robot's arm and link lengths
    bool isReachable(const Vector3D& target) const;

    // Clamps angles of joints to make sure they stay within their limits
    void clampAngles(std::vector<float>& angles) const;

    // Additional utility methods
    void setTolerances(float value);  // Set tolerance
    void setMaxIterations(int value); // Set max iterations
};

#endif // IKSOLVER_HPP
