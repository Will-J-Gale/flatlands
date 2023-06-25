#pragma once
#include <cmath>
#include <limits>

namespace Math
{
    const float FLOAT_MAX = std::numeric_limits<float>::max();
    const float FLOAT_MIN = -FLOAT_MAX;
    const float EPSILON = 1e-3;
    const float PI = 3.1415926535;

    inline float radians(float degrees)
    {
        return degrees * (PI / 180.0f);
    }

    inline bool nearlyEqual(float a, float b, float epsilon=EPSILON)
    {
        return std::abs(a - b) < epsilon;
    }

    inline float toRadians(float degrees)
    {
        return degrees * (PI / 180);
    }

    inline float toDegrees(float radians)
    {
        return radians * (180 / PI);
    }

    inline float clamp(float value, float min, float max)
    {
        return std::max(std::min(value, max), min);
    }
}