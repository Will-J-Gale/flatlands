#pragma once

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

	Line getLine(Vector2 position, float rotation);

    float width = 0;
    Vector2 start = Vector2(0, 0);
    Vector2 end = Vector2(0, 0);
};