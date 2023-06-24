#pragma once
#include <vector>
#include <colliders/Collider.h>
#include <colliders/CollisionPoints.h>
#include <Vector2.h>

class BoxCollider : public Collider
{
public:
    BoxCollider();
    BoxCollider(float width, float height);

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
    
    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::BOX; }
    
    //@TODO Change this to just taken in a Transform*
    std::vector<Vector2> transformPoints(Vector2 position, float rotation);
    std::vector<Vector2> getAxes(float radians);

    float width = 0;
    float height = 0;

private:
    float halfWidth = 0;
    float halfHeight = 0;
};