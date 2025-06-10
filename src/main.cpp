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

// Zet de waardes om naar radialen, en schrijft naar bestand.
void writeToFile(const std::vector<float> &jointAnglesInDegrees, const std::string &filename)
{
    std::ofstream outfile(filename);
    if (!outfile)
    {
        std::cerr << "Kan bestand niet openen voor schrijven: " << filename << std::endl;
        return;
    }

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
    std::cout << "Running IK test...\n";

    // 1. Maak de RobotArm aan
    RobotArm robotArm;

    // 2. Doelpositie opgeven (bijv. binnen bereik van de arm)
    Vector3D target(0.250f, -0.00f, 0.00f);
    Eigen::Matrix3f R;

    // Stel gewenste oriëntatie in (Z wijst omlaag, X en Y zoals normaal)
    R.col(0) = Eigen::Vector3f(0, 0, 0);  // X-as
    R.col(1) = Eigen::Vector3f(1, 0, 0);  // Y-as naar 
    R.col(2) = Eigen::Vector3f(0, 0, 1);  // Z-as omlaag

    std::cout << "Target positie: ";
    target.printVector();

    // 3. Roep inverse kinematica aan
    robotArm.moveTo(target, R); // Deze roept solveIK → solvePositionOnly()

    // 4. Haal de nieuwe joint-hoeken op (geclampte waarden)
    std::vector<float> finalAngles;
    for (const Joint &joint : robotArm.joints)
    {
        finalAngles.push_back(joint.getAngle());
    }

    // 5. Schrijf ze weg naar bestand (in radialen)
    std::string filename = "/home/nout/ros2_ws/joints.txt";
    writeToFile(finalAngles, filename);

    // 6. Print de daadwerkelijke bereikte positie (via FK)
    Vector3D reached = robotArm.getPartialEndEffectorPosition(3);
    // printEndEffectorPosition(reached, "Bereikte positie");

    return 0;
}
