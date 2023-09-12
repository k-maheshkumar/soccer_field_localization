#pragma once

#include "ParticleFilter.hpp"

#include <array>

namespace soccer_field_localization
{
class Localization
{
public:
    Localization(const RobotParams& _robotParams, const std::array<FieldLocation, NUM_LANDMARKS>& markerLocations);

    /*
     *This method intialize the particles around the given intial state.
     *@param intialState provides intial state of the robot
     */
    void init(const RobotState& intialState);

    /*
     *This method predicts state based on given motion input.
     *@param delta provides dx, dy, dtheta of the robot motion
     */
    void motionUpdate(const RobotState& delta);

    /*
    *This method update the robot state based on the observations.
    *@param observations observations[i] provides distance and orientation to the marker
                         with respect to robot position
    */
    void sensorUpdate(const std::vector<MarkerObservation> observations);

    /*
     *This method returns the current state
     *@param state current estimated state of the robot
     */
    void getState(RobotState& state);

    /*
     * This method is to visualize the particles of the filter.
     *@param drawFunction For each particles, the given draw function will be called
                          the user can implement own visualization
     */
    void visualize(const std::function<void(const RobotState& state)>& drawFunction) const;

private:
    const MarkerObservation observationNoise;
    ParticleFilter particleFilter;
};
} // namespace soccer_field_localization