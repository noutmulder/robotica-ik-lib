#include <iostream>
#include <ik_solver/Vector3D.hpp>

int main()
{
    Vector3D vector1(0, 0, 0);
    Vector3D vector2(1, 1, 0);

    std::cout << "mag: " << vector1.magnitude() << std::endl;
    std::cout << "dist to v2: " << vector1.distanceTo(vector2) << std::endl;
    vector1.addVector(vector2).printVector();
    vector1.subtractVector(vector2).printVector();

    return 0;
}
