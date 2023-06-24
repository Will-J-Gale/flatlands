#pragma once

#include <Transform.h>
#include <colliders/AABBCollider.h>

//Forward declarations
class CollisionPoints;
class CircleCollider;
class LineCollider;
class BoxCollider;

enum class ColliderType
{
    BOX,
    CIRCLE,
    LINE
};

class Collider
{
public:
    Collider(){};
    virtual AABBCollider GetAABB(Transform* transform) = 0;
    virtual ColliderType GetType() = 0;
};