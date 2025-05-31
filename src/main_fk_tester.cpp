#include "RobotArm.hpp"
#include "Vector3D.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

void printEndEffectorPosition(const Vector3D &position, const std::string &label)
{
    std::cout << label << ": ("
              << position.x << ", "
              << position.y << ", "
              << position.z << ")\n";
}

// 2) Bepaal de initiële positie van de end-effector
void testEndEffectorPosition()
{
    try
    {
        // Maak een robotarm met standaardconfiguratie
        RobotArm robotArm;

        // 1) Print de begin-hoeken
        std::vector<float> angles = robotArm.ik->getJointAngles();
        std::cout << "Initial joint angles:\n";
        for (size_t i = 0; i < angles.size(); ++i)
        {
            std::cout << "  Joint[" << i << "] = " << angles[i] << "°\n";
        }

        // 2) Bepaal de initiële positie van de end-effector
        Vector3D initialPosition = robotArm.getEndEffectorPosition();
        printEndEffectorPosition(initialPosition, "\nInitial End Effector Position");

        // -- Verwachte initiële positie berekenen (bijv. als alles op 0 staat, moet het op de X-as liggen) --
        float expectedX = 0.0f;
        for (const auto &joint : robotArm.joints)
        {
            expectedX += joint.link->length;
        }

        std::cout << "\nExpected End Effector Position: (" << expectedX << ", 0, 0)\n";

        // Controleer of de positie klopt
        if (std::fabs(initialPosition.x - expectedX) >= 0.01)
        {
            throw std::runtime_error("Fout: End Effector positie op de X-as klopt niet!");
        }
        if (initialPosition.y != 0.0f)
        {
            throw std::runtime_error("Fout: End Effector positie op de Y-as klopt niet!");
        }
        if (initialPosition.z != 0.0f)
        {
            throw std::runtime_error("Fout: End Effector positie op de Z-as klopt niet!");
        }

        // 3) Stel een test-stand in
        std::vector<float> testAngles = {90.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        if (robotArm.ik->setJointAngles(testAngles))
        {
        }
        else
        {
            throw std::runtime_error("Fout: Kan de test-hoeken niet instellen, out of range!");
        }

        // 4) Vraag de hoeken opnieuw op en print ze
        angles = robotArm.ik->getJointAngles();
        std::cout << "\nAfter setting test angles:\n";
        for (size_t i = 0; i < angles.size(); ++i)
        {
            std::cout << "  Joint[" << i << "] = " << angles[i] << "°\n";
        }

        // 5) Bepaal de nieuwe positie van de end-effector
        Vector3D newPosition = robotArm.getEndEffectorPosition();
        printEndEffectorPosition(newPosition, "\nNew End Effector Position");

        // Controleer of de positie niet hetzelfde is als de originele
        if (initialPosition.x == newPosition.x &&
            initialPosition.y == newPosition.y &&
            initialPosition.z == newPosition.z)
        {
            throw std::runtime_error("Fout: End Effector positie is niet veranderd!");
        }

        std::cout << "\nTest passed: End Effector position changed successfully!\n";
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "Test gefaald: " << e.what() << std::endl;
    }
}

int main()
{
    std::cout << "Running End Effector Position Test...\n";
    testEndEffectorPosition();
    return 0;
}
