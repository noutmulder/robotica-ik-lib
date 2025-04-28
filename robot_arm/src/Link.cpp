#include "../include/Link.hpp"
#include <iostream>

// Constructor voor de link
Link::Link(float len, float m, Vector3D pos) : length(len), mass(m), position(pos) {}

// Print de linkdetails
void Link::describeLink() {
    std::cout << "Length: " << length << ", Mass: " << mass << ", Position: ";
    position.printVector();
}
