
#include <iostream>
#include <cmath>
#include "IKSolver.hpp"
#include "../../robot_arm/include/RobotArm.hpp"

IKSolver::IKSolver(RobotArm *arm, float tolerance, int maxIterations)
    : arm(arm), tolerance(tolerance), maxIterations(maxIterations) {}

std::vector<float> IKSolver::solveIK(const Vector3D &target)
{
    std::vector<float> result(6, 0.0f); // [joint1, ..., joint6]

    // Eerst positie oplossen met joint 1–3
    solvePositionOnly(target, result);

    // Later: oriëntatie oplossen met joint 4–6 (bv. zodat grijper goed staat)
    solveOrientationOnly(target, result);

    std::cout << "\nSamenvatting:\n";
    std::cout << "  Doelpositie:     " << target << "\n";
    std::cout << "  Bereikte positie: " << getEndEffector(result) << "\n";

    return result;
}

void IKSolver::solvePositionOnly(const Vector3D &target, std::vector<float> &result)
{
    float delta = 0.25f; // stapgrootte in graden
    int stagnantIterations = 0;
    float lastDistance = std::numeric_limits<float>::max();

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        Vector3D current = arm->getPartialEndEffectorPosition(3);
        float distance = target.subtractVector(current).magnitude();

        if (std::abs(distance - lastDistance) > 1e-5)
        {
            std::cout << "[Iter " << iter << "] afstand tot doel: " << distance << std::endl;
            lastDistance = distance;
            stagnantIterations = 0;
        }
        else
        {
            stagnantIterations++;
        }

        if (distance < tolerance)
        {
            std::cout << "Doel bereikt binnen tolerantie" << std::endl;
            break;
        }

        if (stagnantIterations >= 50 && delta > 0.01f)
        {
            delta = std::max(delta / 2.0f, 0.01f);
            std::cout << "Delta verlaagd naar " << delta << std::endl;
            stagnantIterations = 0;
        }

        for (int i = 0; i < 3; ++i)
        {
            float originalAngle = arm->joints[i].getAngle();

            // test +delta
            float testAnglePlus = originalAngle + delta;
            if (testAnglePlus >= arm->joints[i].getMinAngle() && testAnglePlus <= arm->joints[i].getMaxAngle())
            {
                arm->joints[i].setAngle(testAnglePlus);
                float testDistPlus = target.subtractVector(arm->getPartialEndEffectorPosition(3)).magnitude();

                // test -delta
                float testAngleMinus = originalAngle - delta;
                float testDistMinus = std::numeric_limits<float>::max();
                if (testAngleMinus >= arm->joints[i].getMinAngle() && testAngleMinus <= arm->joints[i].getMaxAngle())
                {
                    arm->joints[i].setAngle(testAngleMinus);
                    testDistMinus = target.subtractVector(arm->getPartialEndEffectorPosition(3)).magnitude();
                }

                // kies de beste richting
                if (testDistPlus < testDistMinus && testDistPlus < distance)
                {
                    arm->joints[i].setAngle(testAnglePlus);
                }
                else if (testDistMinus < distance)
                {
                    arm->joints[i].setAngle(testAngleMinus);
                }
                else
                {
                    arm->joints[i].setAngle(originalAngle);
                }
            }
            else
            {
                // test -delta alleen als +delta ongeldig is
                float testAngleMinus = originalAngle - delta;
                if (testAngleMinus >= arm->joints[i].getMinAngle() && testAngleMinus <= arm->joints[i].getMaxAngle())
                {
                    arm->joints[i].setAngle(testAngleMinus);
                    float testDistMinus = target.subtractVector(arm->getPartialEndEffectorPosition(3)).magnitude();
                    if (testDistMinus < distance)
                    {
                        arm->joints[i].setAngle(testAngleMinus);
                    }
                    else
                    {
                        arm->joints[i].setAngle(originalAngle);
                    }
                }
                else
                {
                    arm->joints[i].setAngle(originalAngle); // geen geldige beweging mogelijk
                }
            }
        }
    }

    // sla resultaat op
    result.clear();
    for (int i = 0; i < 3; ++i)
    {
        result.push_back(arm->joints[i].getAngle());
    }
}

void IKSolver::solveOrientationOnly(const Vector3D &target, std::vector<float> &result)
{
    // TODO: implementeer oriëntatie-oplossing voor joint 4–6

    // Voor nu: laat deze joints op 0 staan
    result[3] = 0.0f;
    result[4] = 0.0f;
    result[5] = 0.0f;

    std::cout << "[solveOrientationOnly] Joint 4-6 op 0° gelaten (nog niet geïmplementeerd)\n";
}

Vector3D IKSolver::getEndEffector(const std::vector<float> &jointAngles) const
{
    for (int i = 0; i < 3; ++i)
    {
        arm->joints[i].setAngle(jointAngles[i]);
    }
    return arm->getPartialEndEffectorPosition(3);
}
