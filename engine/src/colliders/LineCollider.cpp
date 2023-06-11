#include <colliders/LineCollider.h>
#include <colliders/CollisionAlgorithms.h>
#include <cmath>

#define PI 3.14159

LineCollider::LineCollider()
{
}

LineCollider::LineCollider(Vector2 start, Vector2 end)
{
    this->start = start;
    this->end = end;
    this->width = Vector2::distance(start, end);
}

CollisionPoints LineCollider::TestCollision(
		Transform* transform,
		Collider* collider,
		Transform* colliderTransform)
{
    return collider->TestCollision(colliderTransform, this, transform);
}

CollisionPoints LineCollider::TestCollision(
		Transform* transform,
		CircleCollider* circleCollider,
		Transform* circleTransform)
{
    return collisionAlgorithms::FindLineCircleCollisionPoints(
        this, transform,
        circleCollider, circleTransform
    );
}

CollisionPoints LineCollider::TestCollision(
		Transform* transform,
		LineCollider* lineCollider,
		Transform* lineTransform)
{
    return collisionAlgorithms::FindLineLineCollisionPoints(
        this, transform,
        lineCollider, lineTransform
    );
}

CollisionPoints LineCollider::TestCollision(
		Transform* transform,
		BoxCollider* boxCollider,
		Transform* boxTransform)
{
    return collisionAlgorithms::FindLineBoxCollisionPoints(
        this, transform,
        boxCollider, boxTransform
    );
}

Line LineCollider::getLine(Vector2 position, float radians)
{
    Line line;
    float radius = width / 2.0f;

    line.start.x = position.x + (radius * std::cos(radians));
    line.start.y = position.y + (radius * std::sin(radians));

    line.end.x = position.x + (radius * std::cos(radians + PI));
    line.end.y = position.y + (radius * std::sin(radians + PI));

    return line;
}