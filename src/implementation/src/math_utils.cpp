#include "implementation/math_utils.hpp"

#include <cmath>


double MathUtils::map(const double originalNumber, const double in_min, const double in_max, const double out_min, const double out_max)
{
    return (originalNumber - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


double MathUtils::pythagoreanTheorem(double a, double b, double c)
{
    return std::sqrt(a * a + b * b + c * c);
}