#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP


const double PI = 3.141592653589793238463;


class MathUtils
{
public:

    static double map(const double originalNumber, const double in_min, const double in_max, const double out_min, const double out_max);

    static double pythagoreanTheorem(double a, double b, double c);
};



#endif