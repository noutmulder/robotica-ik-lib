#ifndef JOINT_HPP
#define JOINT_HPP

#include "Link.hpp"

// Klasse voor een gewricht van de robotarm (verbinding tussen twee links)
class Joint
{
public:
    float angle;    // Hoek van het gewricht
    float minAngle; // Minimum hoek
    float maxAngle; // Maximum hoek

    Vector3D origin; // van URDF
    Vector3D rpy;    // van URDF
    Vector3D axis;   // bv. (0, 0, 1)

    Link *link; // Verwijst naar de link die met dit gewricht is verbonden

    // Constructor voor het gewricht
    Joint(float ang, float minAng, float maxAng, Link *l);

    // Constructor met volledige URDF-informatie
    Joint(float ang, float minAng, float maxAng, Link *l,
          const Vector3D &origin, const Vector3D &rpy, const Vector3D &axis);

    // Print de joint details
    void describeJoint();

    // Zet de hoek van het gewricht, als binnen de grenzen
    void setAngle(float newAngle);

    float getAngle() const;

    float getMinAngle() const;
    float getMaxAngle() const;
};

#endif
