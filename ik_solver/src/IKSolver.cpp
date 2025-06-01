#include <iostream>
#include <cmath>
#include "IKSolver.hpp"
#include "IKLogging.hpp"
#include "../../robot_arm/include/RobotArm.hpp"

IKSolver::IKSolver(RobotArm *arm, float tolerance, int maxIterations)
    : arm(arm), tolerance(tolerance), maxIterations(maxIterations) {}

std::vector<float> IKSolver::solveIK(const Vector3D &target, const Vector3D &desiredZ)
{
    std::vector<float> result(6, 0.0f);

    // Eerst alleen de positie (joint 1–3)
    solvePositionOnly(target, result);

    Vector3D posJoints1to3 = arm->getPartialEndEffectorPosition(3);

    // Daarna oriëntatie (joint 4–6)
    solveOrientationOnly(desiredZ, result);

    Vector3D finalPos = arm->getEndEffectorPosition();

#if IK_LOG_SUMMARY
    std::cout << "\nSamenvatting:\n";
    std::cout << "  Doelpositie:                  " << target << "\n";
    std::cout << "  Bereikte positie (joints 1-3): " << posJoints1to3 << "\n";
    std::cout << "  Bereikte positie (volledig):   " << finalPos << "\n";
#endif

    return result;
}

void IKSolver::solvePositionOnly(const Vector3D &target, std::vector<float> &result)
{
    float delta = 0.25f;
    int stagnantIterations = 0;
    float lastDistance = std::numeric_limits<float>::max();

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        Vector3D current = arm->getEndEffectorPosition();
        float distance = target.subtractVector(current).magnitude();

#if IK_LOG_POSITION
        if (std::abs(distance - lastDistance) > 1e-5)
            std::cout << "[Iter " << iter << "] afstand tot doel: " << distance << "\n";
#endif

        if (std::abs(distance - lastDistance) > 1e-5)
        {
            lastDistance = distance;
            stagnantIterations = 0;
        }
        else
        {
            stagnantIterations++;
        }

        if (distance < tolerance)
        {
#if IK_LOG_POSITION
            std::cout << "[IK] Doel bereikt binnen tolerantie\n";
#endif
            break;
        }

        if (stagnantIterations >= 50 && delta > 0.01f)
        {
            delta = std::max(delta / 2.0f, 0.01f);
#if IK_LOG_POSITION
            std::cout << "[IK] Delta verlaagd naar " << delta << "\n";
#endif
            stagnantIterations = 0;
        }

        for (int i = 0; i < 3; ++i)
        {
            float originalAngle = arm->joints[i].getAngle();
            float bestDistance = distance;
            float bestAngle = originalAngle;

            for (float dir : {+delta, -delta})
            {
                float testAngle = originalAngle + dir;
                if (testAngle >= arm->joints[i].getMinAngle() && testAngle <= arm->joints[i].getMaxAngle())
                {
                    arm->joints[i].setAngle(testAngle);
                    float testDist = target.subtractVector(arm->getEndEffectorPosition()).magnitude();

                    if (testDist < bestDistance)
                    {
                        bestDistance = testDist;
                        bestAngle = testAngle;
                    }
                }
            }

            arm->joints[i].setAngle(bestAngle);
        }
    }

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
        Eigen::Matrix4f tf = arm->getEndEffectorTransform();
        Eigen::Vector3f currentZ = tf.block<3, 1>(0, 2);
        float alignment = desiredZ.toEigen().normalized().dot(currentZ.normalized());

#if IK_LOG_ORIENTATION
        if (alignment > bestDot + 1e-4f)
            std::cout << "[Orientation Iter " << iter << "] dot(Z, desired): " << alignment << "\n";
#endif

        if (alignment > bestDot + 1e-4f)
        {
            bestDot = alignment;
            stagnant = 0;
        }
        else
        {
            stagnant++;
        }

        if (alignment > 0.999f)
        {
#if IK_LOG_ORIENTATION
            std::cout << "[Orientation] Doel georiënteerd. Alignment: " << alignment << "\n";
#endif
            break;
        }

        if (stagnant > 50 && delta > 0.01f)
        {
            delta = std::max(delta * 0.5f, 0.01f);
            if (delta != lastDelta)
            {
#if IK_LOG_ORIENTATION
                std::cout << "[Orientation] Delta verlaagd naar " << delta << "\n";
#endif
                lastDelta = delta;
            }
            stagnant = 0;
        }

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

            arm->joints[i].setAngle(chosenAngle);
        }
    }

    for (int i = 3; i < 6; ++i)
        result[i] = arm->joints[i].getAngle();
}

Vector3D IKSolver::getEndEffector(const std::vector<float> &jointAngles) const
{
    for (size_t i = 0; i < jointAngles.size(); ++i)
        arm->joints[i].setAngle(jointAngles[i]);

    return arm->getEndEffectorPosition();
}
