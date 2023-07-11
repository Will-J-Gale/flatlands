#pragma once

#include <math/Vector2.h>

struct ClosestVertexProjection
{
    Vector2 vertex;
    Vector2 projectedPoint;
    float distance;
};

struct Line
{
    Line(){};
    Line(Vector2 start, Vector2 end);
    Vector2 closestPointOnLine(Vector2 point);
    ClosestVertexProjection closestVertexOnLine(std::vector<Vector2> vertices);

    Vector2 start;
    Vector2 end;
};