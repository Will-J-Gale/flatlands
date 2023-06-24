#pragma once

#include <Vector2.h>

struct AABBCollider
{
    AABBCollider(){};
    AABBCollider(Vector2 min, Vector2 max)
    {
        this->min = min;
        this->max = max;
    }

    Vector2 min;
    Vector2 max;
};