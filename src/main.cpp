#include <iostream>
#include <ik_solver/Vector3D.hpp>
#include "ik_solver/IKSolver.hpp"

int main()
{
    Vector3D vector1(3, 3, 3);
    Vector3D vector2(4, 5, 6);

    std::cout << "mag: " << vector1.magnitude() << std::endl;
    std::cout << "dist to v2: " << vector1.distanceTo(vector2) << std::endl;
    std::cout << "addition: ";
    vector1.addVector(vector2).printVector();
    std::cout << "subtraction: ";
    vector1.subtractVector(vector2).printVector();
    std::cout << "crossproduct: ";
    vector1.crossProduct(vector2).printVector();
    std::cout << "dotproduct: " << vector1.dotProduct(vector2) << std::endl;

    return 0;
}
