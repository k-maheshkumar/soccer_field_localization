#pragma once
#include <math.h>

namespace soccer_field_localization
{
/*
 * Normalize angle the angel between -pi and pi
 *@param angle input angle to be normalized
 *@return angle normalized angle
 */
double normalizeAngle(const double angle);

/*
 * Computes normalized angle difference between two input angles
 *@param lhs input angle1
 *@param rhs input angle2
 *@return angle returns the ouput of normalizeAngle(lhs - rhs), for more details see method normalizeAngle()
 */
double computeAngleDifference(const double lhs, const double rhs);
} // namespace soccer_field_localization
