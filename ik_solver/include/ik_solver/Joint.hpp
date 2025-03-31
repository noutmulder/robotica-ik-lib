#pragma once

class Link;  // Forward declaration van Link

class Joint {
private:
    float angle;
    float minAngle;
    float maxAngle;
    Link* link;  // Pointer to Link

public:
    // Constructor
    Joint(float minAngle, float maxAngle, float initialAngle, Link* link);

    // Getter for the angle of the joint
    float getAngle() const;

    // Setter for the angle of the joint, returns false if the angle is out of bounds
    bool setAngle(float newAngle);

    // Function to rotate the joint by a certain amount
    void rotate(float delta);
};