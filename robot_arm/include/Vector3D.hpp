#pragma once
#include <vector>
#include <Eigen/Dense>
#include <ostream>

class Vector3D
{
public:
    float x, y, z;
    Vector3D(float x = 0, float y = 0, float z = 0);

    float magnitude() const;
    Vector3D normalize() const;
    float distanceTo(Vector3D) const;
    Vector3D subtractVector(const Vector3D &) const;
    Vector3D addVector(const Vector3D &) const;
    Vector3D scaleVector(float factor) const;
    float dotProduct(const Vector3D &other) const;
    Vector3D crossProduct(const Vector3D &other) const;
    void setVector(float newX, float newY, float newZ);
    void printVector() const;

    // Converteer tussen Eigen en Vector3D
    Eigen::Vector3f toEigen() const;
    static Vector3D fromEigen(const Eigen::Vector3f &vec);

    friend std::ostream &operator<<(std::ostream &os, const Vector3D &v);
};
