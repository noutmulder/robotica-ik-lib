#ifndef JOINT_HPP
#define JOINT_HPP

#include "Link.hpp"

// Klasse voor een gewricht van de robotarm (verbinding tussen twee links)
class Joint {
public:
    float angle;     // Hoek van het gewricht
    float minAngle;  // Minimum hoek
    float maxAngle;  // Maximum hoek
    Link* link;      // Verwijst naar de link die met dit gewricht is verbonden

    // Constructor voor het gewricht
    Joint(float ang, float minAng, float maxAng, Link* l);

    // Print de joint details
    void describeJoint();

    // Zet de hoek van het gewricht, als binnen de grenzen
    void setAngle(float newAngle);
};

#endif
