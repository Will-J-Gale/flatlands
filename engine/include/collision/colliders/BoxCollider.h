#pragma once
#include <vector>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <collision/CollisionPoints.h>
#include <Vector2.h>
#include <Line.h>

class BoxCollider : public ConvexPolygonCollider
{
public:
    BoxCollider(float width, float height);
    float GetRotationalInertia(float mass) override;

    float width = 0;
    float height = 0;

private:
    float halfWidth = 0;
    float halfHeight = 0;
};