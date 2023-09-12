#include "Localization.hpp"
#include "ParticleFilter.hpp"
#include "PfModel.hpp"
#include "robot_defs.h"

namespace soccer_field_localization
{

const static int NUMBER_OF_PARTICLES = 10000;
const static double PARTICLE_SLOW_RECOVERY_FACTOR = 0.001;
const static double PARTICLE_FAST_RECOVERY_FACTOR = 0.1;

Localization::Localization(const RobotParams& _robotParams,
                           const std::array<FieldLocation, NUM_LANDMARKS>& _markerLocations) :
    observationNoise{-1, _robotParams.sensor_noise_distance, _robotParams.sensor_noise_orientation},
    particleFilter{
        ParticleFilterParams{static_cast<size_t>(FIELD_LENGTH / (2.0 * PIXELS_PER_METER)),
                             static_cast<size_t>(FIELD_WIDTH / (2.0 * PIXELS_PER_METER)), NUMBER_OF_PARTICLES,
                             PARTICLE_SLOW_RECOVERY_FACTOR, PARTICLE_SLOW_RECOVERY_FACTOR},
        _markerLocations,
        std::move(std::unique_ptr<soccer_field_localization::PfModel>{
            std::make_unique<soccer_field_localization::PfModel>(_robotParams.odom_noise_rotation_from_rotation,
                                                                 _robotParams.odom_noise_rotation_from_translation,
                                                                 _robotParams.odom_noise_translation_from_translation,
                                                                 _robotParams.odom_noise_translation_from_rotation)})}
{}

void Localization::init(const RobotState& intialState)
{
    particleFilter.init(intialState, observationNoise);
}

void Localization::motionUpdate(const RobotState& delta)
{
    particleFilter.motionUpdate(delta);
}

void Localization::sensorUpdate(const std::vector<MarkerObservation> observations)
{
    particleFilter.sensorUpdate(observations, observationNoise);
}

void Localization::getState(RobotState& state)
{
    state = particleFilter.getMean();
}

void Localization::visualize(const std::function<void(const RobotState& state)>& drawFunction) const
{
    particleFilter.visualize(drawFunction);
}

} // namespace soccer_field_localization