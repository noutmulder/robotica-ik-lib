#include "../include/Vector3D.hpp"
#include <iostream>
#include <cmath>

// constructor
Vector3D::Vector3D(float xVal, float yVal, float zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

// method to calculate the magnitude of the vector
float Vector3D::magnitude() const
{
    return sqrt(x * x + y * y + z * z);
}

// set length of vector to 0
Vector3D Vector3D::normalize() const
{
    float mag = magnitude();
    if (mag == 0)
    {
        return Vector3D(0, 0, 0); // voorkom delen door 0
    }
    return Vector3D(x / mag, y / mag, z / mag);
}

// calculate distance to vector
float Vector3D::distanceTo(Vector3D toVector) const
{
    return sqrt((x - toVector.x) * (x - toVector.x) + (y - toVector.y) * (y - toVector.y) + (z - toVector.z) * (z - toVector.z));
}

// subtract vector from vector
Vector3D Vector3D::addVector(const Vector3D& secondVector){
    return Vector3D(x + secondVector.x, y + secondVector.y, z + secondVector.z);
}

// add vector to vector
Vector3D Vector3D::subtractVector(const Vector3D& secondVector){
    return Vector3D(x - secondVector.x, y - secondVector.y, z - secondVector.z);
}

// print a vector for debugging
void Vector3D::printVector() {
    std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;

}

// scales a vector
Vector3D Vector3D::scaleVector(float factor){
    return Vector3D(x*factor,y*factor,z*factor);
}

// calculate dotproduct
float Vector3D::dotProduct(const Vector3D& other) const{
    return x*other.x + y * other.y + z * other.z;
}

// calculate crossproduct
Vector3D Vector3D::crossProduct(const Vector3D& other) const{
    return Vector3D(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

void Vector3D::setVector(float newX, float newY, float newZ) {
    x = newX;
    y = newY;
    z = newZ;
}


Eigen::Vector3f Vector3D::toEigen() const {
    return Eigen::Vector3f(x, y, z);
}

Vector3D Vector3D::fromEigen(const Eigen::Vector3f& vec) {
    return Vector3D(vec.x(), vec.y(), vec.z());
}


