#include "RobotArm.hpp"
#include "Vector3D.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cassert>
#include <Eigen/Dense>

void printEndEffectorPosition(const Vector3D &position, const std::string &label)
{
    std::cout << label << ": ("
              << position.x << ", "
              << position.y << ", "
              << position.z << ")\n";
}

// zet de waardens om naar radialen, en schrijft naar bestand.
void writeToFile(const std::vector<float> &jointAnglesInDegrees, const std::string &filename)
{
    std::ofstream outfile(filename);
    if (!outfile)
    {
        std::cerr << "Kan bestand niet openen voor schrijven: " << filename << std::endl;
        return;
    }

    // Zet elke hoek om van graden naar radialen en schrijf naar bestand
    for (size_t i = 0; i < jointAnglesInDegrees.size(); ++i)
    {
        float radians = jointAnglesInDegrees[i] * (M_PI / 180.0f);
        outfile << radians;
        if (i != jointAnglesInDegrees.size() - 1)
            outfile << ",";
    }
    outfile << std::endl;

    outfile.close();
}

int main()
{
    std::cout << "Running writing test...\n";

    // Maak een RobotArm object aan
    RobotArm robotArm;

    // Definieer de hoeken voor alle joints (allemaal 0.0)
    std::vector<float> fixedJointAngles = {-30.0f, -30.0f, -30.0f, -30.0f, -30.0f, -30.0f};
    // std::vector<float> fixedJointAngles = {90.0f, 0.0f, -0.0f, 0.0f, 0.0f, 0.0f};

    // Zet deze hoeken in de robotarm
    if (robotArm.ik->setJointAngles(fixedJointAngles))
    {
        std::string filename = "/home/nout/ros2_ws/joints.txt";

        // Haal de actuele hoeken op uit de robot
        std::vector<float> clampedAngles; // clamped betekend dat de hoeken binnen de limieten zijn
        for (const Joint &joint : robotArm.joints)
        {
            clampedAngles.push_back(joint.getAngle());
        }

        writeToFile(clampedAngles, filename);
    } // Belangrijk: deze functie valideert én zet de hoeken

    // Bereken en print de end-effector positie
    // Vector3D endEffector = robotArm.getEndEffectorPosition();
    Vector3D endEffector = robotArm.getPartialEndEffectorPosition(3);
    printEndEffectorPosition(endEffector, "End-effector positie");

    return 0;
}
