#include "ik_solver/Joint.hpp"
#include "ik_solver/Link.hpp"

Joint::Joint(float minAngle, float maxAngle, float initialAngle, Link *link) : minAngle(minAngle), maxAngle(maxAngle), angle(initialAngle), link(link) {}

//getter for angle
float Joint::getAngle() const
{
    return angle;
}

// sets the angle of the joint, returns false if the angle is out of bounds
bool Joint::setAngle(float newAngle)
{
    if (newAngle < minAngle || newAngle > maxAngle)
    {
        return false;
    }
    angle = newAngle;
    return true;
}

// rotates the joint by amount delta
void Joint::rotate(float delta)
{
    setAngle(angle + delta);
}