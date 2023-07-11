#pragma once

#include <math/Vector2.h>

struct AABB
{
    AABB(){};
    AABB(Vector2 min, Vector2 max)
    {
        this->min = min;
        this->max = max;

        this->width = this->max.x - this->min.x;
        this->height = this->max.y - this->min.y;
    }

    Vector2 min;
    Vector2 max;

    float width = 0;
    float height = 0;
};