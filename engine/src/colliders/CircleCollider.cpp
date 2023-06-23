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
    return CollisionAlgorithms::FindCircleCircleCollisionPoints(
        this, transform,
        circleCollider, circleTransform
    );
}

CollisionPoints CircleCollider::TestCollision(
		Transform* transform,
		LineCollider* lineCollider,
		Transform* lineTransform)
{
    return CollisionAlgorithms::FindLineCircleCollisionPoints(
        lineCollider, lineTransform,
        this, transform
    );
}

CollisionPoints CircleCollider::TestCollision(
		Transform* transform,
		BoxCollider* boxCollider,
		Transform* boxTransform)
{
    return CollisionAlgorithms::FindBoxCircleCollision(
        boxCollider, boxTransform,
        this, transform
    );
}