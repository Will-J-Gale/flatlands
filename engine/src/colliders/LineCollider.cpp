#include <cmath>

#include <colliders/LineCollider.h>
#include <colliders/CollisionAlgorithms.h>
#include <core/Logger.h>

#define PI 3.14159

LineCollider::LineCollider()
{
}

LineCollider::LineCollider(Vector2 position, float width)
{
    _width = width;
    _start = Vector2(-width / 2.0f, 0.0f);
    _end = Vector2(width / 2.0f, 0.0f);
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
    return CollisionAlgorithms::FindLineCircleCollisionPoints(
        this, transform,
        circleCollider, circleTransform
    );
}

CollisionPoints LineCollider::TestCollision(
		Transform* transform,
		LineCollider* lineCollider,
		Transform* lineTransform)
{
    return CollisionAlgorithms::FindLineLineCollisionPoints(
        this, transform,
        lineCollider, lineTransform
    );
}

CollisionPoints LineCollider::TestCollision(
		Transform* transform,
		BoxCollider* boxCollider,
		Transform* boxTransform)
{
    return CollisionAlgorithms::FindLineBoxCollisionPoints(
        this, transform,
        boxCollider, boxTransform
    );
}

std::vector<Vector2> LineCollider::transformPoints(Vector2 position, float radians)
{
    std::vector<Vector2> points;

    Vector2 startRotated = _start.rotate(radians) + position;
    Vector2 endRotated = _end.rotate(radians) + position;
    points.push_back(endRotated);
    points.push_back(startRotated);

    return points;
}


AABBCollider LineCollider::GetAABB(Transform* transform)
{
    return AABBCollider();
}

Vector2 LineCollider::getAxis(float radians)
{
    Vector2 startRotated = _start.rotate(radians);
    Vector2 endRotated = _end.rotate(radians);

    return endRotated - startRotated;
}