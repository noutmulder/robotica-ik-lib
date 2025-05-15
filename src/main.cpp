#include "RobotArm.hpp"
#include "Vector3D.hpp"
#include <iostream>

int main()
{
    // Maak een robotarm met standaardconfiguratie
    RobotArm robotArm;

    // 1) Print de begin-hoeken (ze staan in de joints, standaard 0° of wat je init hebt)
    std::vector<float> angles =  robotArm.ik->getJointAngles();
    std::cout << "Initial joint angles:\n";
    for (size_t i = 0; i < angles.size(); ++i) {
        std::cout << "  Joint[" << i << "] = " << angles[i] << "°\n";
    }

    // 2) Print de begin-end-effector positie
    Vector3D endEffector = robotArm.getEndEffectorPosition();
    std::cout << "\nEnd Effector Position: ("
              << endEffector.x << ", "
              << endEffector.y << ", "
              << endEffector.z << ")\n";

    // 3) Stel een test-stand in
    std::vector<float> testAngles = {45.0f, -30.0f, 15.0f, 0.0f, 20.0f, -10.0f};
    robotArm.ik->setJointAngles(testAngles);

    // 4) Vraag de hoeken opnieuw op en print ze
    angles = robotArm.ik->getJointAngles();  
    std::cout << "\nAfter setting test angles:\n";
    for (size_t i = 0; i < angles.size(); ++i) {
        std::cout << "  Joint[" << i << "] = " << angles[i] << "°\n";
    }

    // 5) En de nieuwe end-effector positie
    endEffector = robotArm.getEndEffectorPosition();
    std::cout << "\nEnd Effector Position: ("
              << endEffector.x << ", "
              << endEffector.y << ", "
              << endEffector.z << ")\n";

    return 0;
}
