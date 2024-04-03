#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP


const double PI = 3.141592653589793238463;


class MathUtils
{
public:

    /// @brief Maps one value to another value
    /// @param originalNumber The original value
    /// @param in_min The minimum value of the original value
    /// @param in_max The maximum value of the original value
    /// @param out_min The minimum value of the new value
    /// @param out_max The maximum value of the new value
    /// @return The new value
    static double map(const double originalNumber, const double in_min, const double in_max, const double out_min, const double out_max);

    /// @brief Applies the pythagoreanTheorem in a 3D space
    /// @param a The distance in the first dimension
    /// @param b The distance in the second dimension
    /// @param c The distance in the third dimension
    /// @return The total distance
    static double pythagoreanTheorem(const double a, const double b, const double c);
};



#endif