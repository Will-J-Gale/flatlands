#include <colliders/CircleCollider.h>
#include <colliders/CollisionAlgorithms.h>

CircleCollider::CircleCollider() : Collider()
{
    
}

CircleCollider::CircleCollider(float radius) : Collider()
{
    this->radius = radius;
}

CollisionPoints CircleCollider::TestCollision(
		Transform* transform,
		Collider* collider,
		Transform* colliderTransform)
{
    return collider->TestCollision(colliderTransform, this, transform);
}

CollisionPoints CircleCollider::TestCollision(
		Transform* transform,
		CircleCollider* circleCollider,
		Transform* circleTransform)
{
    return collisionAlgorithms::FindCircleCircleCollisionPoints(
        this, transform,
        circleCollider, circleTransform
    );
}

CollisionPoints CircleCollider::TestCollision(
		Transform* transform,
		LineCollider* lineCollider,
		Transform* lineTransform)
{
    return collisionAlgorithms::FindLineCircleCollisionPoints(
        lineCollider, lineTransform,
        this, transform
    );
}

CollisionPoints CircleCollider::TestCollision(
		Transform* transform,
		BoxCollider* boxCollider,
		Transform* boxTransform)
{
    return collisionAlgorithms::FindBoxCircleCollision(
        boxCollider, boxTransform,
        this, transform
    );
}