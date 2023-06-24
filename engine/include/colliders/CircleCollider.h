#pragma once

#include "Collider.h"
#include <colliders/CollisionPoints.h>

class CircleCollider : public Collider
{
public:
    CircleCollider();
    CircleCollider(float radius);

    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::CIRCLE; }
    
    float radius = 0;
};