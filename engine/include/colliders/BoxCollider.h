#pragma once
#include <vector>
#include <colliders/Collider.h>
#include <colliders/CollisionPoints.h>
#include <Vector2.h>

class BoxCollider : public Collider
{
public:
    BoxCollider();
    BoxCollider(float width, float height);

    AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::BOX; }
    
    //@TODO Change this to just taken in a Transform*
    std::vector<Vector2> transformPoints(Vector2 position, float rotation);
    std::vector<Vector2> getAxes(float radians);

    float width = 0;
    float height = 0;

private:
    float halfWidth = 0;
    float halfHeight = 0;
};