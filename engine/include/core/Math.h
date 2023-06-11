#pragma once
#include <cmath>

namespace math
{
    const float PI = 3.1415926535;

    inline float radians(float degrees)
    {
        return degrees * (PI / 180.0f);
    }
}