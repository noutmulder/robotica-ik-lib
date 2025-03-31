#pragma once
#include "ik_solver/Vector3D.hpp" 

class IKSolver
{
private:
    Vector3D position;

public:
    IKSolver();
    std::vector<float> solveIK(const Vector3D& target);
    Vector3D getPosition() const;
};
