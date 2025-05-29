#include "../include/Link.hpp"
#include <iostream>

// Constructor voor de link
Link::Link(float len, float m) : length(len), mass(m) {}

// Print de linkdetails
void Link::describeLink() const {
    std::cout << "Length: " << length << ", Mass: " << mass << std::endl;
}
