#pragma once

#include "robot_defs.h"
#include <random>

namespace soccer_field_localization
{
class PfModel
{
public:
    /*
     *Particle filter model with motion and measurement model
     *@param _odomNoiseRotationFromRotation The noise in the odometric rotation estimate from robot rotational movement
     *@param _odomNoiseRotationFromTranslation The noise in the odometric rotation estimate from robot translational
     *movement
     *@param _odomNoiseTranslationFromTranslation The noise in the odometric translation estimate from robot
     *translational movement
     *@param _odomNoiseTranslationFromRotation The noise in the odometric translation estimate from robot rotational
     *movement
     */
    PfModel(const double _odomNoiseRotationFromRotation, const double _odomNoiseRotationFromTranslation,
            const double _odomNoiseTranslationFromTranslation, const double _odomNoiseTranslationFromRotation);

    /*
     *Predict the robot state based on the odometry information with the chosen motion model
     *@param previous Previous state of the robot
     *@param delta represents the movement made by the robot in x, y and theta
     */
    RobotState predict(const RobotState& previous, const RobotState& delta);
    /*
     *Estimates the particle observation based on the robot seen marker
     *@param particle Current particle state
     *@param marker actual observed marker in the map frame
     */
    MarkerObservation update(const RobotState& particle, const FieldLocation marker);

private:
    std::random_device rd;
    std::mt19937 mersenne{rd()};
    double odomNoiseRotationFromRotation, odomNoiseRotationFromTranslation, odomNoiseTranslationFromTranslation,
        odomNoiseTranslationFromRotation;
};
} // namespace soccer_field_localization
