#include <collision/colliders/CapsuleCollider.h>
#include <core/Math.h>

CapsuleCollider::CapsuleCollider(float width, float height)
{
    this->width = width;
    this->height = height;
    this->halfHeight = height / 2.0f;
    this->radius = width / 2.0f;

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

AABBCollider CapsuleCollider::GetAABB(Transform* transform)
{
    //@TODO This is incorrect, the AAB is bigger when the capsule is rotated
    Vector2 min = Vector2(Math::FLOAT_MAX, Math::FLOAT_MAX);
    Vector2 max = Vector2(Math::FLOAT_MIN, Math::FLOAT_MIN);

    for(Vector2& vertex : aabbVertices)
    {
        Vector2 transformedVertex = vertex.rotate(transform->rotation) + transform->position;

        if(transformedVertex.x < min.x)
            min.x = transformedVertex.x;

        if(transformedVertex.x > max.x)
            max.x = transformedVertex.x;

        if(transformedVertex.y < min.y)
            min.y = transformedVertex.y;

        if(transformedVertex.y > max.y)
            max.y = transformedVertex.y;
    }

    return AABBCollider(min, max);
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