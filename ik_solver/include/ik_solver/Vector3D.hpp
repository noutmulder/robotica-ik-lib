#pragma once

class Vector3D {
    public:
        float x, y, z;
        Vector3D(float x = 0, float y = 0, float z = 0);

    float magnitude() const;
    Vector3D normalize() const;
    float distanceTo(Vector3D) const;
    Vector3D subtractVector(const Vector3D&);
    Vector3D addVector(const Vector3D&);
    Vector3D scaleVector(float factor);
    void printVector();
};
    