#pragma once

#include "Collider.h"
#include <colliders/CollisionPoints.h>

class CircleCollider : public Collider
{
public:
    CircleCollider();
    CircleCollider(float radius);

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
		BoxCollider* boxCollider,
		Transform* lineTransform) override;

    float radius = 0;
};