#pragma once

#include "PfModel.hpp"
#include "robot_defs.h"

#include <functional>
#include <memory>
#include <random>
#include <vector>

namespace soccer_field_localization
{
class ParticleFilter
{
public:
    using Weight = double;
    using ParticleSet = std::vector<std::pair<RobotState, Weight>>;

    /*
     *Particle filter constructor
     *@param _numberOfParticles number of particles to approximate the posterior distribution
     *@param _markerLocations location of markers with respect to map
     *@param _pfModel motion model of the robot
     *@param _sensorNoiseDistance The sensor noise in range measurement
     *@param _sensorNoiseOrientation The sensor noise in bearing measurement
     */
    ParticleFilter(const size_t _numberOfParticles, const std::array<FieldLocation, NUM_LANDMARKS>& _markerLocations,
                   std::unique_ptr<PfModel> _pfModel);

    /*
     *This method intialize the particles around the given intial state.
     *@param intialState provides intial state of the robot
     */
    void init(const RobotState& intialState, const MarkerObservation& observationNoise);

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
    void sensorUpdate(const std::vector<MarkerObservation> observations, const MarkerObservation& observationNoise);

    /*
     *This method returns the current state
     */
    const RobotState& getMean() const;

    /*
     * This method is to visualize the particles of the filter.
     *@param drawFunction For each particles, the given draw function will be called
                          the user can implement own visualization
     */
    void visualize(const std::function<void(const RobotState& state)>& drawFunction) const;

private:
    /// Generate sample robot state particle
    std::pair<RobotState, Weight> generateRandomSample() const;

    /* Computes the unnormalized measurement probability for the given state
     * @param state The assumed robot state
     * @param measurements The current robot observation of the markers
     */
    double computeParticleWeight(const RobotState& state, const std::vector<MarkerObservation> observations,
                                 const MarkerObservation& observationNoise) const;

    /* Normalizes computed weights with the total weight from computeParticleWeight()
     * @param totalWeight total weight of all particles
     */
    void normalizeWeights(const double totalWeight);

    /// Resamples the particles based on the measurement/observation
    void resample();

    /// updates the mean with the best particle belief
    void updateMean();

    mutable std::mt19937 mersenne{std::random_device{}()};
    std::array<FieldLocation, NUM_LANDMARKS> markerLocations;
    std::unique_ptr<PfModel> pfModel;

    // We need two particle set.
    // A predicted set and the posterior
    std::array<ParticleSet, 2> particles;
    uint8_t priorSetIndex{0};
    uint8_t updatedSetIndex{1};

    /// Current estimated robot state
    RobotState mean;

    double weightSlow{0};
    double weightFast{0};
};
} // namespace soccer_field_localization