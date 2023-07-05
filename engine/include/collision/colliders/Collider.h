#pragma once

#include <Transform.h>
#include <collision/colliders/AABBCollider.h>

//Forward declarations
class CollisionPoints;
class CircleCollider;
class LineCollider;
class BoxCollider;

enum class ColliderType
{
    POLYGON,
    CIRCLE,
    CAPSULE
};

class Collider
{
public:
    Collider(){};
    virtual AABBCollider GetAABB(Transform* transform) = 0;
    virtual ColliderType GetType() = 0;
    virtual float GetRotationalInertia(float mass) = 0;
};