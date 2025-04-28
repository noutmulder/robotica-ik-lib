#include <iostream>
#include "Vector3D.hpp"
#include "Link.hpp"
#include "Joint.hpp"
#include "RobotArm.hpp"


int main() {
    // Testen van de Vector3D klasse
    Vector3D vector1(3, 3, 3);
    Vector3D vector2(4, 5, 6);
    std::cout << "mag: " << vector1.magnitude() << std::endl;
    std::cout << "dist to v2: " << vector1.distanceTo(vector2) << std::endl;
    std::cout << "addition: ";
    vector1.addVector(vector2).printVector();
    std::cout << "subtraction: ";
    vector1.subtractVector(vector2).printVector();
    std::cout << "crossproduct: ";
    vector1.crossProduct(vector2).printVector();
    std::cout << "dotproduct: " << vector1.dotProduct(vector2) << std::endl;

    // Testen van de Link klasse
    Link link1(5.0, 1.0, Vector3D(0, 0, 0));  // Maak een link met lengte 5, massa 1, positie (0,0,0)
    Link link2(3.0, 0.8, Vector3D(5, 0, 0));  // Maak een andere link met lengte 3, massa 0.8, positie (5,0,0)

    // Beschrijf de links
    std::cout << "\nLink 1 Details:" << std::endl;
    link1.describeLink();
    std::cout << "\nLink 2 Details:" << std::endl;
    link2.describeLink();

    // Testen van de Joint klasse
    Joint joint1(30, -90, 90, &link1);  // Maak een joint met hoek 30°, min hoek -90° en max hoek 90° voor link1
    Joint joint2(45, -90, 90, &link2);  // Maak een joint met hoek 45° voor link2

    // Beschrijf de joints
    std::cout << "\nJoint 1 Details:" << std::endl;
    joint1.describeJoint();
    std::cout << "\nJoint 2 Details:" << std::endl;
    joint2.describeJoint();

    // Verander de hoek van joint1
    joint1.setAngle(60);  // Verander de hoek naar 60°
    std::cout << "\nJoint 1 Details after changing angle:" << std::endl;
    joint1.describeJoint();  // Beschrijf de joint opnieuw na de wijziging

    // Testen van een ongeldige hoek (hoek buiten bereik)
    joint2.setAngle(120);  // Probeer de hoek in te stellen buiten het bereik (max 90°)
    
    return 0;
}
