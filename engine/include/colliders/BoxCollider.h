#pragma once
#include <vector>
#include <colliders/PolygonCollider.h>
#include <colliders/CollisionPoints.h>
#include <Vector2.h>
#include <Line.h>

class BoxCollider : public PolygonCollider
{
public:
    BoxCollider();
    BoxCollider(float width, float height);

    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::POLYGON; }
    
    std::vector<Vector2> transformPoints(Transform* transform) override;
    std::vector<Vector2> getAxes(float radians) override;
    std::vector<Line> getEdges(Transform* transform) override;
    float GetRotationalInertia(float mass) override;

    float width = 0;
    float height = 0;

private:
    float halfWidth = 0;
    float halfHeight = 0;
};