#pragma once
#include <cmath>
#include <limits>

namespace Math
{
    const float FLOAT_MAX = std::numeric_limits<float>::max();
    const float FLOAT_MIN = -FLOAT_MAX;
    const float PI = 3.1415926535;

    inline float radians(float degrees)
    {
        return degrees * (PI / 180.0f);
    }
}