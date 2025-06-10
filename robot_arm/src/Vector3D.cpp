#include "../include/Vector3D.hpp"
#include <iostream>
#include <cmath>

// Constructor: maakt een vector met de opgegeven x, y en z waardes
Vector3D::Vector3D(float xVal, float yVal, float zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

// Geeft de lengte van de vector terug (afstand vanaf oorsprong)
float Vector3D::magnitude() const
{
    return sqrt(x * x + y * y + z * z);
}

// Geeft een nieuwe vector met dezelfde richting, maar lengte 1
Vector3D Vector3D::normalize() const
{
    float mag = magnitude();
    if (mag == 0)
    {
        return Vector3D(0, 0, 0); // voorkom delen door 0
    }
    return Vector3D(x / mag, y / mag, z / mag);
}

// Bereken de afstand tot een andere vector
float Vector3D::distanceTo(Vector3D toVector) const
{
    return sqrt((x - toVector.x) * (x - toVector.x) +
                (y - toVector.y) * (y - toVector.y) +
                (z - toVector.z) * (z - toVector.z));
}

// Tel er een vector bij op (component-gewijs)
Vector3D Vector3D::addVector(const Vector3D &secondVector) const
{
    return Vector3D(x + secondVector.x, y + secondVector.y, z + secondVector.z);
}

// Trek een vector eraf (component-gewijs)
Vector3D Vector3D::subtractVector(const Vector3D &secondVector) const
{
    return Vector3D(x - secondVector.x, y - secondVector.y, z - secondVector.z);
}

// Print de vector op het scherm, bijvoorbeeld: (1.0, 2.0, 3.0)
void Vector3D::printVector() const
{
    std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
}

// Schaal de vector (verleng of verkort met factor)
Vector3D Vector3D::scaleVector(float factor) const
{
    return Vector3D(x * factor, y * factor, z * factor);
}

// Bepaal hoe "gelijk gericht" twee vectoren zijn (hoeveel ze in dezelfde richting wijzen)
float Vector3D::dotProduct(const Vector3D &other) const
{
    return x * other.x + y * other.y + z * other.z;
}

// Geeft een vector loodrecht op beide invoervectoren (3D rotatie-as)
Vector3D Vector3D::crossProduct(const Vector3D &other) const
{
    return Vector3D(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x);
}

// Zet nieuwe waardes voor deze vector
void Vector3D::setVector(float newX, float newY, float newZ)
{
    x = newX;
    y = newY;
    z = newZ;
}

// Zet deze vector om naar een Eigen 3D vector (voor compatibiliteit met Eigen)
Eigen::Vector3f Vector3D::toEigen() const
{
    return Eigen::Vector3f(x, y, z);
}

// Zet een Eigen vector om naar deze Vector3D-structuur
Vector3D Vector3D::fromEigen(const Eigen::Vector3f &vec)
{
    return Vector3D(vec.x(), vec.y(), vec.z());
}

// Maakt het mogelijk om Vector3D te printen met cout
std::ostream &operator<<(std::ostream &os, const Vector3D &v)
{
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}
