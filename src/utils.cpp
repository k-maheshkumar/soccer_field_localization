#include "utils.hpp"
#include <ctime>
#include <iostream>

namespace soccer_field_localization
{
// normalize angle between -M_PI and M_PI
double normalizeAngle(const double angle)
{
    const double result{fmod(angle + M_PI, 2.0 * M_PI)};
    if (result <= 0.0)
    {
        return result + M_PI;
    }
    return result - M_PI;
}
double computeAngleDifference(const double lhs, const double rhs)
{
    return normalizeAngle(lhs - rhs);
}
} // namespace soccer_field_localization
