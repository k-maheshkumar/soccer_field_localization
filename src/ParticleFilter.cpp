#include "ParticleFilter.hpp"
#include "robot_defs.h"
#include "utils.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>

namespace soccer_field_localization
{
ParticleFilter::ParticleFilter(const size_t _numberOfParticles,
                               const std::array<FieldLocation, NUM_LANDMARKS>& _markerLocations,
                               std::unique_ptr<PfModel> _pfModel) :
    markerLocations{_markerLocations},
    pfModel{std::move(_pfModel)}
{
    particles[priorSetIndex].reserve(_numberOfParticles);
    particles[updatedSetIndex].reserve(_numberOfParticles);

    assert(_numberOfParticles > 0);

    for (uint32_t i = 0; i < _numberOfParticles; i++)
    {
        const auto particle = generateRandomSample();
        particles[priorSetIndex].emplace_back(particle);
        particles[updatedSetIndex].emplace_back(particle);
    }
}

std::pair<RobotState, ParticleFilter::Weight> ParticleFilter::generateRandomSample() const
{
    const double halfLength{FIELD_LENGTH / (2.0 * PIXELS_PER_METER)};
    const double halfWidth{FIELD_WIDTH / (2.0 * PIXELS_PER_METER)};

    auto random_x = std::uniform_real_distribution<>(-halfLength, halfLength);
    auto random_y = std::uniform_real_distribution<>(-halfWidth, halfWidth);
    auto random_yaw = std::uniform_real_distribution<>(-M_PI, M_PI);
    auto random_weight = std::uniform_real_distribution<>(0, 1.0);

    RobotState state;
    state.x = random_x(mersenne);
    state.y = random_y(mersenne);
    state.theta = random_yaw(mersenne);
    const double weight = random_weight(mersenne);
    return {state, weight};
}

void ParticleFilter::init(const RobotState& intialState, const MarkerObservation& observationNoise)
{
    const auto sample{[&](const double _mean, const double _sigma) -> double {
        std::normal_distribution<double> distribution(_mean, _sigma);
        return distribution(mersenne);
    }};

    for (auto& particle : particles[priorSetIndex])
    {
        particle.first.x = sample(intialState.x, observationNoise.distance);
        particle.first.y = sample(intialState.y, observationNoise.distance);
        particle.first.theta = sample(particle.first.theta, observationNoise.orientation);
    }

    resample();
    updateMean();
}

void ParticleFilter::motionUpdate(const RobotState& delta)
{
    const double halfLength{FIELD_LENGTH / (2.0 * PIXELS_PER_METER)};
    const double halfWidth{FIELD_WIDTH / (2.0 * PIXELS_PER_METER)};

    for (auto& [particle, weight] : particles[priorSetIndex])
    {
        particle = pfModel->predict(particle, delta);
        particle.x = fmax(-halfLength, particle.x);
        particle.y = fmax(-halfWidth, particle.y);
        particle.x = fmin(halfLength, particle.x);
        particle.y = fmin(halfWidth, particle.y);
    }
}

void ParticleFilter::sensorUpdate(const std::vector<MarkerObservation> observations,
                                  const MarkerObservation& observationNoise)
{
    double totalWeight{0.0};
    for (auto& [particle, weight] : particles[priorSetIndex])
    {
        auto newWeight = computeParticleWeight(particle, observations, observationNoise);
        weight *= newWeight;
        totalWeight += weight;
    }

    particles[updatedSetIndex] = particles[priorSetIndex];
    normalizeWeights(totalWeight);
    resample();
    updateMean();

    while (particles[updatedSetIndex].size() < particles[priorSetIndex].size())
    {
        particles[updatedSetIndex].push_back(generateRandomSample());
    }

    particles[priorSetIndex] = particles[updatedSetIndex];
}

double ParticleFilter::computeParticleWeight(const RobotState& particle,
                                             const std::vector<MarkerObservation> observations,
                                             const MarkerObservation& observationNoise) const
{
    const double halfLength{FIELD_LENGTH / (2.0 * PIXELS_PER_METER)};
    const double halfWidth{FIELD_WIDTH / (2.0 * PIXELS_PER_METER)};

    if (particle.x < -halfLength || particle.y < -halfWidth || particle.x > halfLength || particle.y > halfWidth)
    {
        return 1.0 / static_cast<double>(particles[priorSetIndex].size());
    }

    double weight{1.0};

    const auto gaussianProbability{[](const double _mean, const double _sigma) {
        return std::max(exp(-0.5 * pow(_mean / _sigma, 2)) / (_sigma * sqrt(2.0 * M_PI)), 1e-6);
    }};

    const auto sample{[&](const double sigma) -> double {
        std::normal_distribution<double> distribution(0.0, sigma);
        return distribution(mersenne);
    }};

    for (const auto& observation : observations)
    {
        if (static_cast<size_t>(observation.markerIndex) > markerLocations.size())
        {
            throw std::runtime_error("Invalid observation marker size: " + std::to_string(observation.markerIndex));
        }

        const auto& marker{markerLocations[static_cast<size_t>(observation.markerIndex)]};
        auto estimatedObservation{pfModel->update(particle, marker)};
        estimatedObservation.distance += sample(observationNoise.distance);
        estimatedObservation.orientation =
            normalizeAngle(estimatedObservation.orientation + observationNoise.orientation);

        double prob =
            gaussianProbability(fabs(estimatedObservation.distance - observation.distance), observationNoise.distance) *
            gaussianProbability(fabs(estimatedObservation.orientation - observation.orientation),
                                observationNoise.orientation);

        weight *= prob;
    }

    return weight;
}

void ParticleFilter::normalizeWeights(const double totalWeight)
{
    for (auto& [_, weight] : particles[updatedSetIndex])
    {
        if (weight > 0.0)
        {
            weight /= totalWeight;
        }
        else
        {
            weight /= static_cast<double>(particles[updatedSetIndex].size());
        }
    }
}

// reference: https://www.youtube.com/watch?v=wNQVo6uOgYA
void ParticleFilter::resample()
{

    double maxWeight = std::numeric_limits<double>::min();
    const auto& updatedParticles = particles[updatedSetIndex];

    ParticleSet resampledParticles;
    resampledParticles.reserve(updatedParticles.size());

    for (size_t i{0}; i < updatedParticles.size(); ++i)
    {
        if (updatedParticles[i].second > maxWeight)
        {
            maxWeight = updatedParticles[i].second;
        }
    }

    auto randomDouble{std::uniform_real_distribution<>(0.0, 2.0 * maxWeight)};
    size_t bestIndex{std::uniform_int_distribution<size_t>(0, updatedParticles.size() - 1)(mersenne)};

    double beta{0.0};

    for (size_t index{0}; index < updatedParticles.size(); ++index)
    {
        beta += randomDouble(mersenne);

        while (beta > updatedParticles[bestIndex].second)
        {
            beta -= updatedParticles[bestIndex].second;
            bestIndex = (bestIndex + 1) % updatedParticles.size();
        }
        resampledParticles.push_back(updatedParticles[bestIndex]);
    }

    particles[updatedSetIndex] = resampledParticles;
}

void ParticleFilter::updateMean()
{
    const auto& updatedParticles{particles[updatedSetIndex]};
    double maxWeight{std::numeric_limits<double>::min()};

    for (size_t index{0}; index < updatedParticles.size(); ++index)
    {
        if (updatedParticles[index].second > maxWeight)
        {
            maxWeight = updatedParticles[index].second;
            mean = updatedParticles[index].first;
            mean.theta = normalizeAngle(mean.theta);
        }
    }
}

void ParticleFilter::visualize(const std::function<void(const RobotState& state)>& drawFunction) const
{
    for (const auto& [particle, _] : particles[priorSetIndex])
    {
        drawFunction(particle);
    }
}

const RobotState& ParticleFilter::getMean() const
{
    return mean;
}
} // namespace soccer_field_localization
