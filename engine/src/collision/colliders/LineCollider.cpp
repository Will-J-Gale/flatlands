#include <cmath>

#include <collision/colliders/LineCollider.h>
#include <collision/CollisionAlgorithms.h>
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

std::vector<Vector2> LineCollider::transformPoints(Transform* transform)
{
    Vector2 position = transform->position;
    float radians = transform->rotation;
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

float LineCollider::GetRotationalInertia(float mass)
{
    return mass;
}