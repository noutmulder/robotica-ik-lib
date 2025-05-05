#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP

class InverseKinematics;

#include <vector>

#define MAX_ITERATIONS 1000
#define TOLERANCE 0.01

class IKSolver {
public:
    InverseKinematics* arm;       // Verwijzing naar de arm (pointer naar InverseKinematics)
    float tolerance;              // Hoe dicht bij de doelpositie
    int maxIterations;            // Maximale aantal iteraties

    // Constructor voor IKSolver, initialiseert met arm, tolerantie en maximaal aantal iteraties
    IKSolver(InverseKinematics* arm, float tolerance = TOLERANCE, int maxIterations = MAX_ITERATIONS);

    // Lost de inverse kinematica op voor de gegeven doelpositie
    std::vector<float> solveIK(const Vector3D& target);

    // Controleert of het doel bereikbaar is op basis van de robotarm en de lengtes van de links
    bool isReachable(const Vector3D& target) const;

    // Beperk de hoeken van gewrichten om ervoor te zorgen dat ze binnen hun grenzen blijven
    void clampAngles(std::vector<float>& angles) const;

    // Extra hulpfuncties
    void setTolerances(float value);  // Stel de tolerantiewaarde in
    void setMaxIterations(int value); // Stel het maximale aantal iteraties in
};

#endif // IKSOLVER_HPP
