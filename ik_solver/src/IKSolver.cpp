
#include <iostream>
#include <cmath>
#include "IKSolver.hpp"

IKSolver::IKSolver(InverseKinematics* arm, float tolerance, int maxIterations)
    : arm(arm), tolerance(tolerance), maxIterations(maxIterations) {}

Vector3D IKSolver::getEndEffector(const std::vector<float>& jointAngles) const {
    float x = 0.0f, y = 0.0f, z = 0.0f;  // Beginpositie van de arm
    float currentAngle = 0.0f;  // Huidige hoek begint op 0

    // Loop over alle gewrichten
    for (size_t i = 0; i < jointAngles.size(); ++i) {
        Link* currentLink = arm->joints[i].link;  // Gebruik een pointer
        float length = currentLink->length;  // Haal de lengte van de link op

        // Update de huidige hoek (afhankelijk van de joint hoek)
        currentAngle += jointAngles[i];

        // Bereken de bijdrage van de huidige link aan de eindpositie
        x += length * cos(currentAngle);
        y += length * sin(currentAngle);
    }

    return Vector3D(x, y, z);  // Retourneer de uiteindelijke positie van de eind-effector
}


std::vector<float> IKSolver::solveIK(const Vector3D& target) {
    std::vector<float> jointAngles;

    // Placeholder: Je kunt een numerieke methode zoals Newton's method of de Jacobiaan gebruiken
    // Hier geven we tijdelijk een dummy waarde voor de joint hoeken terug.
    
    jointAngles.push_back(0.0f);  // Voeg een dummy joint angle toe voor testdoeleinden.
    
    return jointAngles;
}
