#include "RobotArm.hpp"
#include "Vector3D.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cassert>

void printEndEffectorPosition(const Vector3D& position, const std::string& label) {
    std::cout << label << ": ("
              << position.x << ", "
              << position.y << ", "
              << position.z << ")\n";
}

void writeToFile(const std::vector<float>& jointAngles, const std::string& filename) {
    std::ofstream outfile(filename);
    if (!outfile) {
        std::cerr << "Kan bestand niet openen voor schrijven: " << filename << std::endl;
        return;
    }

    // Schrijf alle joint waarden, gescheiden door komma's
    for (size_t i = 0; i < jointAngles.size(); ++i) {
        outfile << jointAngles[i];
        if (i != jointAngles.size() - 1)
            outfile << ",";
    }
    outfile << std::endl;

    outfile.close();
}


int main() {
    std::cout << "Running writing test...\n";
    // testEndEffectorPosition();
    std::vector<float> fixedJointAngles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::string filename = "/home/nout/ros2_ws/joints.txt";
    writeToFile(fixedJointAngles, filename);

    return 0;
}
