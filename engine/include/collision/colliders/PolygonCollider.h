#pragma once
#include <vector>
#include <Transform.h>
#include <collision/colliders/Collider.h>
#include <collision/CollisionPoints.h>
#include <Vector2.h>
#include <Line.h>

class PolygonCollider : public Collider
{
public:
    PolygonCollider(){};

    virtual std::vector<Vector2> transformPoints(Transform* transform) = 0;
    virtual std::vector<Vector2> getAxes(float radians) = 0;
    virtual std::vector<Line> getEdges(Transform* transform) = 0;
};