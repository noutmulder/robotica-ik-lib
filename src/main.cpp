#include "RobotArm.hpp"
#include "Vector3D.hpp"
#include <iostream>

int main() {
    // Maak een robotarm met standaardconfiguratie
    RobotArm robotArm;

    // Haal en toon de positie van de end-effector
    Vector3D endEffector = robotArm.getEndEffectorPosition();
    std::cout << "End Effector Position: ("
              << endEffector.x << ", "
              << endEffector.y << ", "
              << endEffector.z << ")\n";

    return 0;
}
