#include "../include/Joint.hpp"
#include <iostream>

// Constructor zonder positionele data (voor backward compatibility)
Joint::Joint(float ang, float minAng, float maxAng, Link* l)
    : angle(ang), minAngle(minAng), maxAngle(maxAng), origin(0, 0, 0), rpy(0, 0, 0), axis(0, 0, 1), link(l) {}

// Nieuwe constructor met volledige URDF data
Joint::Joint(float ang, float minAng, float maxAng, Link* l,
             const Vector3D& origin, const Vector3D& rpy, const Vector3D& axis)
    : angle(ang), minAngle(minAng), maxAngle(maxAng), origin(origin), rpy(rpy), axis(axis), link(l) {}

// Print de jointdetails, inclusief URDF gerelateerde informatie
void Joint::describeJoint() {
    std::cout << "Joint Angle: " << angle 
              << ", Min Angle: " << minAngle 
              << ", Max Angle: " << maxAngle << std::endl;

try
{
    std::cout << "Origin: "; origin.printVector();
    std::cout << "RPY: "; rpy.printVector();
    std::cout << "Axis: "; axis.printVector();
}
catch(const std::exception& e)
{
    std::cerr << e.what() << '\n';
}

}

// Zet de hoek van het gewricht binnen de min/max grenzen
void Joint::setAngle(float newAngle) {
    if (newAngle >= minAngle && newAngle <= maxAngle) {
        angle = newAngle;
    } else {
        std::cout << "Angle out of range!" << std::endl;
    }
}

// Haal de huidige hoek van het gewricht op
float Joint::getAngle() const {
    return angle;
}
