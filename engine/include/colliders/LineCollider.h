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

    CollisionPoints TestCollision(
		Transform* transform,
		Collider* collider,
		Transform* colliderTransform) override;
 
	CollisionPoints TestCollision(
		Transform* transform,
		CircleCollider* circleCollider,
		Transform* circleTransform) override;

    CollisionPoints TestCollision(
		Transform* transform,
		LineCollider* lineCollider,
		Transform* lineTransform) override;
	
	CollisionPoints TestCollision(
		Transform* transform,
		BoxCollider* lineCollider,
		Transform* lineTransform) override;

    AABBCollider GetAABB(Transform* transform) override;

    std::vector<Vector2> transformPoints(Vector2 position, float radians);
    Vector2 getAxis(float radians);

private:
    float _width = 0;
    Vector2 _start = Vector2(0, 0);
    Vector2 _end = Vector2(0, 0);
};