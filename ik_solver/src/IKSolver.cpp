#include <iostream>
#include <cmath>
#include "IKSolver.hpp"
#include "IKLogging.hpp"
#include "../../robot_arm/include/RobotArm.hpp"

// Constructor: slaat robotarm + instellingen op
IKSolver::IKSolver(RobotArm *arm, float tolerance, int maxIterations)
    : arm(arm), tolerance(tolerance), maxIterations(maxIterations) {}

// Hoofdfunctie voor inverse kinematica
// 1. Los positie op met joints 1–3
// 2. Pas daarna de oriëntatie aan met joints 4–6
std::vector<float> IKSolver::solveIK(const Vector3D &target, const Eigen::Matrix3f &R_des)
{
    std::vector<float> result(6, 0.0f);

    solvePositionOnly(target, result); // eerst positie

    Vector3D posJoints1to3 = arm->getPartialEndEffectorPosition(3);

    solveOrientationOnly(R_des, result); // dan oriëntatie

    Vector3D finalPos = arm->getEndEffectorPosition();

#if IK_LOG_SUMMARY
    std::cout << "\nSamenvatting:\n";
    std::cout << "  Doelpositie:                  " << target << "\n";
    std::cout << "  Bereikte positie (joints 1-3): " << posJoints1to3 << "\n";
    std::cout << "  Bereikte positie (volledig):   " << finalPos << "\n";
#endif

    return result;
}

// Optimaliseert alleen de positie van de end-effector (joints 1–3)
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

        // Als de afstand niet verbetert, tel dat bij stagnant op
        if (std::abs(distance - lastDistance) > 1e-5)
        {
            lastDistance = distance;
            stagnantIterations = 0;
        }
        else
        {
            stagnantIterations++;
        }

        // Stop als we binnen de tolerantie zitten
        if (distance < tolerance)
        {
#if IK_LOG_POSITION
            std::cout << "[IK] Doel bereikt binnen tolerantie\n";
#endif
            break;
        }

        // Verlaag stapgrootte als er te lang geen verbetering is
        if (stagnantIterations >= 50 && delta > 0.01f)
        {
            delta = std::max(delta / 2.0f, 0.01f);
#if IK_LOG_POSITION
            std::cout << "[IK] Delta verlaagd naar " << delta << "\n";
#endif
            stagnantIterations = 0;
        }

        // Zoek per joint (1–3) de beste richting om afstand te verkleinen
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

    // Zet resultaat (alleen joints 1–3)
    result.clear();
    for (int i = 0; i < 3; ++i)
    {
        result.push_back(arm->joints[i].getAngle());
    }
}

// Optimaliseert de volledige oriëntatie van de end-effector (joints 4–6)
void IKSolver::solveOrientationOnly(const Eigen::Matrix3f &R_des, std::vector<float> &result)
{
    float delta = 1.0f;
    int stagnant = 0;
    float bestScore = -100.0f;
    float lastDelta = delta;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // Huidige oriëntatie van de end-effector
        Eigen::Matrix4f tf = arm->getEndEffectorTransform();
        Eigen::Vector3f x_current = tf.block<3, 1>(0, 0);
        Eigen::Vector3f y_current = tf.block<3, 1>(0, 1);
        Eigen::Vector3f z_current = tf.block<3, 1>(0, 2);

        // Gewenste assen (uit R_des matrix)
        Eigen::Vector3f x_target = R_des.block<3, 1>(0, 0);
        Eigen::Vector3f y_target = R_des.block<3, 1>(0, 1);
        Eigen::Vector3f z_target = R_des.block<3, 1>(0, 2);

        // Bepaal hoe goed alle assen uitgelijnd zijn (dotproduct)
        float score =
            x_current.normalized().dot(x_target.normalized()) +
            y_current.normalized().dot(y_target.normalized()) +
            z_current.normalized().dot(z_target.normalized());

#if IK_LOG_ORIENTATION
        if (score > bestScore + 1e-4f)
            std::cout << "[Iter " << iter << "] alignment score: " << score << "\n";
#endif

        // Reset stagnatie als er verbetering is
        if (score > bestScore + 1e-4f)
        {
            bestScore = score;
            stagnant = 0;
        }
        else
        {
            stagnant++;
        }

        // Stop als maximale score bereikt is
        if (score > 2.99f)
        {
#if IK_LOG_ORIENTATION
            std::cout << "[Orientation] Volledige oriëntatie bereikt. Score: " << score << "\n";
#endif
            break;
        }

        // Verminder stapgrootte als we vastzitten
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

        // Test elke joint (4–6) met kleine rotaties
        for (int i = 3; i < 6; ++i)
        {
            float original = arm->joints[i].getAngle();
            float bestLocalScore = bestScore;
            float chosenAngle = original;

            for (float direction : {+delta, -delta})
            {
                float testAngle = original + direction;
                if (testAngle < arm->joints[i].getMinAngle() || testAngle > arm->joints[i].getMaxAngle())
                    continue;

                arm->joints[i].setAngle(testAngle);

                Eigen::Matrix4f tfTest = arm->getEndEffectorTransform();
                Eigen::Vector3f xTest = tfTest.block<3, 1>(0, 0);
                Eigen::Vector3f yTest = tfTest.block<3, 1>(0, 1);
                Eigen::Vector3f zTest = tfTest.block<3, 1>(0, 2);

                float testScore =
                    xTest.normalized().dot(x_target.normalized()) +
                    yTest.normalized().dot(y_target.normalized()) +
                    zTest.normalized().dot(z_target.normalized());

                if (testScore > bestLocalScore)
                {
                    bestLocalScore = testScore;
                    chosenAngle = testAngle;
                }
            }

            arm->joints[i].setAngle(chosenAngle);
        }
    }

#if IK_LOG_ORIENTATION || IK_LOG_SUMMARY
    Eigen::Matrix4f tfFinal = arm->getEndEffectorTransform();
    Eigen::Matrix3f R_final = tfFinal.block<3, 3>(0, 0);

    std::cout << "\n[Orientation Result] Bereikte oriëntatie:\n";
    std::cout << R_final << "\n";

    Eigen::Matrix3f R_err = R_final.transpose() * R_des;
    Eigen::AngleAxisf aa(R_err);

    std::cout << "[Orientation Result] Rotatiefout:\n";
    std::cout << "  Draaias  = (" << aa.axis().transpose() << ")\n";
    std::cout << "  Hoek     = " << aa.angle() * (180.0 / M_PI) << " graden\n";
#endif

    // Zet de eindwaarden voor joints 4–6 terug in result
    for (int i = 3; i < 6; ++i)
        result[i] = arm->joints[i].getAngle();
}

// Berekent de eindpositie van de grijper na het instellen van alle hoeken
Vector3D IKSolver::getEndEffector(const std::vector<float> &jointAngles) const
{
    for (size_t i = 0; i < jointAngles.size(); ++i)
        arm->joints[i].setAngle(jointAngles[i]);

    return arm->getEndEffectorPosition();
}
