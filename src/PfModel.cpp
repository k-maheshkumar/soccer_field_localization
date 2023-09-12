#include "PfModel.hpp"
#include "utils.hpp"
#include <cassert>
namespace soccer_field_localization
{
PfModel::PfModel(const double _odomNoiseRotationFromRotation, const double _odomNoiseRotationFromTranslation,
                 const double _odomNoiseTranslationFromTranslation, const double _odomNoiseTranslationFromRotation) :
    odomNoiseRotationFromRotation(_odomNoiseRotationFromRotation),
    odomNoiseRotationFromTranslation(_odomNoiseRotationFromTranslation),
    odomNoiseTranslationFromTranslation(_odomNoiseTranslationFromTranslation),
    odomNoiseTranslationFromRotation(_odomNoiseTranslationFromRotation)
{}

RobotState PfModel::predict(const RobotState& previous, const RobotState& delta)
{
    const auto sample{[&](const double sigma) -> double {
        std::normal_distribution<double> distribution(0.0, sigma);
        return distribution(mersenne);
    }};

    const auto previousYaw{previous.theta};
    const auto deltaRot1{computeAngleDifference(atan2(delta.y, delta.x), previousYaw)};
    const auto deltaTrans{sqrt(pow(delta.x, 2) + pow(delta.y, 2))};
    const auto deltaRot2{computeAngleDifference(delta.theta, deltaRot1)};

    const auto deltaCapRot1{
        computeAngleDifference(deltaRot1, sample(odomNoiseRotationFromRotation * pow(deltaRot1, 2) +
                                                 odomNoiseRotationFromTranslation * pow(deltaTrans, 2)))};

    const auto deltaCapTrans{deltaTrans - sample(odomNoiseTranslationFromTranslation * pow(deltaTrans, 2) +
                                                 odomNoiseTranslationFromRotation * pow(deltaRot1, 2) +
                                                 odomNoiseTranslationFromRotation * pow(deltaRot2, 2))};

    const auto deltaCapRot2 =
        computeAngleDifference(deltaRot2, sample(odomNoiseRotationFromRotation * pow(deltaRot2, 2) +
                                                 odomNoiseRotationFromTranslation * pow(deltaTrans, 2)));

    const auto angle1 = normalizeAngle(previousYaw + deltaCapRot1);
    const auto angle2 = normalizeAngle(angle1 + deltaCapRot2);

    return {previous.x + deltaCapTrans * cos(angle1), previous.y + deltaCapTrans * sin(angle1), angle2};
}

MarkerObservation PfModel::update(const RobotState& particle, const FieldLocation marker)
{
    const double dX = particle.x - marker.x;
    const double dY = particle.y - marker.y;
    double distance = sqrt(pow(dX, 2) + pow(dY, 2));
    const double orientation = computeAngleDifference(particle.theta, atan2(dY, dX));
    return {-1, distance, orientation};
}

} // namespace soccer_field_localization
