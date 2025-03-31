#pragma once

class Joint;  // Forward declaration van Joint

class Link {
private:
    float mass;
    float length;

public:
    // Constructor
    Link(float length, float mass = 0.0f);

    // Getters en setters
    float getMass() const;
    void setMass(float mass);
    float getLength() const;
    void setLength(float length);
};
