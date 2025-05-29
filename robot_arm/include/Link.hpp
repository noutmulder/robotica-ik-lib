#ifndef LINK_HPP
#define LINK_HPP

#include "Vector3D.hpp"

// Klasse voor een link van de robotarm (deel tussen twee gewrichten)
class Link {
public:
    float length;   // Lengte van de link
    float mass;     // Massa van de link

    // Constructor voor de link
    Link(float len, float m);

    // Print de details van de link
    void describeLink() const;
};

#endif
