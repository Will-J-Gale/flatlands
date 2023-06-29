#pragma once
#include <vector>

#include "Collider.h"
#include <collision/CollisionPoints.h>
#include <Line.h>

class LineCollider : public Collider
{
public:
    LineCollider();
    LineCollider(Vector2 start, Vector2 end);
    LineCollider(Vector2 position, float width);

    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::POLYGON; }
    float GetRotationalInertia(float mass) override;

    std::vector<Vector2> transformPoints(Transform* transform);
    Vector2 getAxis(float radians);

private:
    float _width = 0;
    Vector2 _start = Vector2(0, 0);
    Vector2 _end = Vector2(0, 0);
};