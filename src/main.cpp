#include "RobotArm.hpp"
#include "Vector3D.hpp"
#include <iostream>

int main()
{
    // Maak een robotarm met standaardconfiguratie
    RobotArm robotArm;

    Vector3D endEffector = robotArm.getEndEffectorPosition();
    std::cout << "End Effector Position: ("
              << endEffector.x << ", "
              << endEffector.y << ", "
              << endEffector.z << ")\n";

    std::vector<float> testAngles = {45.0f, -30.0f, 15.0f, 0.0f, 20.0f, -10.0f};
    robotArm.ik->setJointAngles(testAngles);

    // Haal en toon de positie van de end-effector
    endEffector = robotArm.getEndEffectorPosition();
    std::cout << "End Effector Position: ("
              << endEffector.x << ", "
              << endEffector.y << ", "
              << endEffector.z << ")\n";

    return 0;
}
