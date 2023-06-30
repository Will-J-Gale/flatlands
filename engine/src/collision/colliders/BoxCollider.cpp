#include <collision/colliders/BoxCollider.h>
#include <collision/CollisionAlgorithms.h>
#include <core/Math.h>
#include <core/Logger.h>

BoxCollider::BoxCollider(float width, float height)
{
    this->width = width;
    this->height = height;

    this->halfWidth = width / 2.0f;
    this->halfHeight = height / 2.0f;

    std::vector<Vector2> vertices = {
        Vector2(-this->halfWidth, -this->halfHeight),
        Vector2(this->halfWidth, -this->halfHeight),
        Vector2(this->halfWidth, this->halfHeight),
        Vector2(-this->halfWidth, this->halfHeight)
    };

    this->setVertices(vertices);
}

float BoxCollider::GetRotationalInertia(float mass)
{
    return (1.0f/12.0f) * mass * ((height*height) + (width*width));
}