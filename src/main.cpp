#include "RobotArm.hpp"
#include "Vector3D.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cassert>
#include <Eigen/Dense>

// Helper-functie om een vector netjes te printen
void printEndEffectorPosition(const Vector3D &position, const std::string &label)
{
    std::cout << label << ": ("
              << position.x << ", "
              << position.y << ", "
              << position.z << ")\n";
}

// Schrijft de gewrichten naar bestand (omgezet naar radialen)
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

    // 1. Maak de robotarm aan (bevat de gewrichten en kinematica)
    RobotArm robotArm;


    // 2. Geef een doelpositie op waar de grijper naartoe moet (in meters)
    float X = 0.15f; // X - voren / achteren
    float Y = 0.15f; // Y - links / rechts
    float Z = 0.30f; // Z - up / down

    Vector3D target(X, Y, Z);

    // 3. Definieer gewenste oriëntatie van de grijper (als rotatiematrix)
    Eigen::Matrix3f R;
    R.col(0) = Eigen::Vector3f(0, 0, 0);    // X-as (niet gebruikt hier)
    R.col(1) = Eigen::Vector3f(-1, 0, 0);   // Y-as wijst naar links
    R.col(2) = Eigen::Vector3f(0, 0, 1);    // Z-as wijst omlaag

    std::cout << "Target positie: ";
    target.printVector();

    // 4. Vraag inverse kinematica aan om de juiste hoeken te vinden
    //    Deze functie lost eerst positie (joints 1–3) en dan oriëntatie (joints 4–6) op
    robotArm.moveTo(target, R);

    // 5. Lees de eindhoeken uit (in graden) en zet ze om naar radialen voor gebruik in ROS
    std::vector<float> finalAngles;
    for (const Joint &joint : robotArm.joints)
    {
        finalAngles.push_back(joint.getAngle());
    }

    // 6. Sla de eindhoeken op in een bestand (radialen, gescheiden door komma's)
    std::string filename = "/home/nout/ros2_ws/joints.txt";
    writeToFile(finalAngles, filename);

    // 7. Print de bereikte positie van de end-effector na alleen positie-oplossing (optioneel)
    // Vector3D reached = robotArm.getPartialEndEffectorPosition(3);
    // printEndEffectorPosition(reached, "Bereikte positie");

    return 0;
}
