#ifndef MATH_H
#define MATH_H

#include <cmath>

/**
 * Fast approximation of atan. Based on https://yal.cc/fast-atan2/.
 */
inline double fatan(double x)
{
    return M_PI_4 * x - x * (fabs(x) - 1) * (0.2447 + 0.0663 * fabs(x));
}

/**
 * Fast approximation of atan2. Based on https://yal.cc/fast-atan2/.
 */
inline double fatan2(double y, double x)
{
    if (x >= 0)  // -pi/2 .. pi/2
    {
        if (y >= 0)  // 0 .. pi/2
        {
            if (y < x)  // 0 .. pi/4
                return fatan(y / x);
            else  // pi/4 .. pi/2
                return M_PI_2 - fatan(x / y);
        }
        else  // -pi/2 .. 0
        {
            if (-y < x)  // -pi/4 .. 0
                return fatan(y / x);
            else  // -pi/2 .. -pi/4
                return -M_PI_2 - fatan(x / y);
        }
    }

    else  // -pi..-pi/2, pi/2..pi
    {
        if (y >= 0)  // pi/2 .. pi
        {
            if (y < -x)  // pi*3/4 .. pi
                return fatan(y / x) + M_PI;
            else  // pi/2 .. pi*3/4
                return M_PI_2 - fatan(x / y);
        }
        else  // -pi .. -pi/2
        {
            if (-y < -x)  // -pi .. -pi*3/4
                return fatan(y / x) - M_PI;
            else  // -pi*3/4 .. -pi/2
                return -M_PI_2 - fatan(x / y);
        }
    }
}

#endif  // MATH_H
