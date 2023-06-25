#pragma once
#include <vector>
#include <colliders/Collider.h>
#include <colliders/CollisionPoints.h>
#include <Vector2.h>
#include <Line.h>

class BoxCollider : public Collider
{
public:
    BoxCollider();
    BoxCollider(float width, float height);

    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::BOX; }
    
    std::vector<Vector2> transformPoints(Transform* transform);
    std::vector<Vector2> getAxes(float radians);
    std::vector<Line> getEdges(Transform* transform);
    float GetRotationalInertia(float mass) override;

    float width = 0;
    float height = 0;

private:
    float halfWidth = 0;
    float halfHeight = 0;
};