
#include <iostream>
#include <cmath>
#include "IKSolver.hpp"
#include "../../robot_arm/include/RobotArm.hpp"

IKSolver::IKSolver(RobotArm *arm, float tolerance, int maxIterations)
    : arm(arm), tolerance(tolerance), maxIterations(maxIterations) {}

std::vector<float> IKSolver::solveIK(const Vector3D &position, const Vector3D &desiredZ)
{
    std::vector<float> result(6, 0.0f); // [joint1, ..., joint6]

    // Eerst positie oplossen met joint 1–3
    solvePositionOnly(position, result);

    // Oriëntatie oplossen met joint 4–6
    solveOrientationOnly(desiredZ, result);

    std::cout << "\nSamenvatting:\n";
    std::cout << "  Doelpositie:                  " << position << "\n";
    std::cout << "  Bereikte positie (joints 1-3): " << getEndEffector(result) << "\n";
    std::cout << "  Bereikte positie (volledig):   " << arm->getEndEffectorPosition() << "\n";

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

void IKSolver::solveOrientationOnly(const Vector3D &desiredZ, std::vector<float> &result)
{
    float delta = 1.0f;
    int stagnant = 0;
    float bestDot = -1.0f;
    float lastDelta = delta;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // Bereken huidige Z-as van end-effector
        Eigen::Matrix4f tf = arm->getEndEffectorTransform();
        Eigen::Vector3f currentZ = tf.block<3, 1>(0, 2);
        float alignment = desiredZ.toEigen().normalized().dot(currentZ.normalized());

        if (alignment > bestDot + 1e-4f)
        {
            std::cout << "[Orientation Iter " << iter << "] dot(Z, desired): " << alignment << "\n";
            bestDot = alignment;
            stagnant = 0;
        }
        else
        {
            stagnant++;
        }

        if (alignment > 0.999f)
        {
            std::cout << "[Orientation] Doel georiënteerd. Alignment: " << alignment << "\n";
            break;
        }

        // Verlaag delta als we te lang stilstaan
        if (stagnant > 50 && delta > 0.01f)
        {
            delta = std::max(delta * 0.5f, 0.01f);
            if (delta != lastDelta)
            {
                std::cout << "[Orientation] Delta verlaagd naar " << delta << "\n";
                lastDelta = delta;
            }
            stagnant = 0;
        }

        // Test voor joints 4–6
        for (int i = 3; i < 6; ++i)
        {
            float original = arm->joints[i].getAngle();
            float bestLocalDot = bestDot;
            float chosenAngle = original;

            for (float direction : {+delta, -delta})
            {
                float testAngle = original + direction;
                if (testAngle < arm->joints[i].getMinAngle() || testAngle > arm->joints[i].getMaxAngle())
                    continue;

                arm->joints[i].setAngle(testAngle);
                Eigen::Vector3f zTest = arm->getEndEffectorTransform().block<3, 1>(0, 2);
                float dot = desiredZ.toEigen().normalized().dot(zTest.normalized());

                if (dot > bestLocalDot)
                {
                    bestLocalDot = dot;
                    chosenAngle = testAngle;
                }
            }

            arm->joints[i].setAngle(chosenAngle); // best gevonden
        }
    }

    // Result opslaan voor joints 4-6
    for (int i = 3; i < 6; ++i)
        result[i] = arm->joints[i].getAngle();
}

Vector3D IKSolver::getEndEffector(const std::vector<float> &jointAngles) const
{
    for (int i = 0; i < 3; ++i)
    {
        arm->joints[i].setAngle(jointAngles[i]);
    }
    return arm->getPartialEndEffectorPosition(3);
}
