#include "../include/Joint.hpp"
#include <iostream>

// Constructor voor het gewricht
Joint::Joint(float ang, float minAng, float maxAng, Link* l) : angle(ang), minAngle(minAng), maxAngle(maxAng), link(l) {}

// Print de jointdetails
void Joint::describeJoint() {
    std::cout << "Joint Angle: " << angle << ", Min Angle: " << minAngle << ", Max Angle: " << maxAngle << std::endl;
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
