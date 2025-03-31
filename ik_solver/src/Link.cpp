#include "ik_solver/Link.hpp"

Link::Link(float length, float mass) : length(length), mass(mass) {}

float Link::getLength() const {
    return length;
}

void Link::setLength(float newLength) {
    length = newLength;
}

float Link::getMass() const {
    return mass;
}

void Link::setMass(float newMass) {
    mass = newMass;
}
