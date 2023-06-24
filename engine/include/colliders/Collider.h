#pragma once

#include <Transform.h>
#include <colliders/AABBCollider.h>

//Forward declarations
class CollisionPoints;
class CircleCollider;
class LineCollider;
class BoxCollider;

enum class ColliderType
{
    BOX,
    CIRCLE,
    LINE
};

class Collider
{
public:
    Collider(){};

    virtual CollisionPoints TestCollision(
		Transform* transform,
		Collider* collider,
		Transform* colliderTransform) = 0;
 
	virtual CollisionPoints TestCollision(
		Transform* transform,
		CircleCollider* sphere,
		Transform* sphereTransform) = 0;
	
	virtual CollisionPoints TestCollision(
		Transform* transform,
		LineCollider* lineCollider,
		Transform* lineTransform) = 0;

	virtual CollisionPoints TestCollision(
		Transform* transform,
		BoxCollider* lineCollider,
		Transform* lineTransform) = 0;
    
    virtual AABBCollider GetAABB(Transform* transform) = 0;
    virtual ColliderType GetType() = 0;
};