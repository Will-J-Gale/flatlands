#pragma once

#include <Vector2.h>

struct Line
{
    Line(){};
    Line(Vector2 start, Vector2 end);
    Vector2 closestPointOnLine(Vector2 point);

    Vector2 start;
    Vector2 end;
};