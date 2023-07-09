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
}

LineCollider::LineCollider(Vector2 start, Vector2 end)
{
    Vector2 center = start + ((end - start) / 2.0f);
    line.start = start - center;
    line.end = end - center;

    this->setVertices({line.start, line.end});
}

Line LineCollider::getLine()
{
    return line;
}

Line LineCollider::transformLine(Transform* transform)
{
    Line transformedLine;
    transformedLine.start = line.start.rotate(transform->rotation) + transform->position;
    transformedLine.end = line.end.rotate(transform->rotation) + transform->position;

    return transformedLine;
}