#include <collision/colliders/CapsuleCollider.h>
#include <core/Math.h>

CapsuleCollider::CapsuleCollider(float width, float height)
{
    this->width = width;
    this->radius = width / 2.0f;
    this->height = height > width ? height : width + 1;
    this->halfHeight = this->height / 2.0f;

    float centerOffset = this->halfHeight - this->radius;
    this->centerLine.start = Vector2(0, centerOffset);
    this->centerLine.end = Vector2(0, -centerOffset);

    this->aabbVertices = {
        Vector2(-radius, -halfHeight),
        Vector2(radius, -halfHeight),
        Vector2(radius, halfHeight),
        Vector2(-radius, halfHeight)
    };
}

AABB CapsuleCollider::GetAABB(Transform* transform)
{
    Line transformedLine = GetCenterLine(transform);
    Vector2 start = transformedLine.start;
    Vector2 end = transformedLine.end;
    AABB aabb;

    aabb.min.x = std::min(start.x, end.x);
    aabb.min.y = std::min(start.y, end.y);
    aabb.max.x = std::max(start.x, end.x);
    aabb.max.y = std::max(start.y, end.y);

    aabb.min += (Vector2::left + Vector2::up) * radius;
    aabb.max += (Vector2::right + Vector2::down) * radius;

    return aabb;
}

float CapsuleCollider::GetRotationalInertia(float mass)
{
    return 0.5f * mass * (radius * radius);
}
float CapsuleCollider::GetWidth()
{
    return this->width;
}

float CapsuleCollider::GetHeight()
{
    return this->height;
}

Line CapsuleCollider::GetCenterLine(Transform* transform)
{
    Line transformedLine; 

    transformedLine.start = centerLine.start.rotate(transform->rotation) + transform->position;
    transformedLine.end = centerLine.end.rotate(transform->rotation) + transform->position;

    return transformedLine;
}