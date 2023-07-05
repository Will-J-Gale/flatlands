#pragma once
#include <vector>
#include <Transform.h>
#include <collision/colliders/Collider.h>
#include <collision/CollisionPoints.h>
#include <Vector2.h>
#include <Line.h>

class CapsuleCollider : public Collider
{
public:
    CapsuleCollider(){};
    CapsuleCollider(float width, float height);

    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::CAPSULE; }
    float GetRotationalInertia(float mass) override;
    float GetWidth();
    float GetHeight();
    Line GetCenterLine(Transform* transform);

private:
    float width = 0.0f;
    float height = 0.0f;
    float halfHeight = 0.0f;
    float radius = 0.0f;
    Line centerLine;
    std::vector<Vector2> aabbVertices;
};