#pragma once
#include <vector>

#include "Collider.h"
#include <colliders/CollisionPoints.h>

struct Line
{
	Vector2 start;
	Vector2 end;
};

class LineCollider : public Collider
{
public:
    LineCollider();
    LineCollider(Vector2 start, Vector2 end);
    LineCollider(Vector2 position, float width);

    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::LINE; }

    std::vector<Vector2> transformPoints(Vector2 position, float radians);
    Vector2 getAxis(float radians);

private:
    float _width = 0;
    Vector2 _start = Vector2(0, 0);
    Vector2 _end = Vector2(0, 0);
};