#pragma once
#include <vector>
#include <Transform.h>
#include <collision/colliders/Collider.h>
#include <collision/CollisionPoints.h>
#include <Vector2.h>
#include <Line.h>

class ConvexPolygonCollider : public Collider
{
public:
    ConvexPolygonCollider(){};
    ConvexPolygonCollider(std::vector<Vector2> vertices);

    virtual void setVertices(std::vector<Vector2> vertices);
    virtual std::vector<Vector2> transformPoints(Transform* transform);
    virtual std::vector<Vector2> getAxes(float radians);
    virtual std::vector<Line> getEdges(Transform* transform);

    virtual AABBCollider GetAABB(Transform* transform) override;
    ColliderType GetType() override { return ColliderType::POLYGON; }
    virtual float GetRotationalInertia(float mass) override;

private:
private:
    std::vector<Vector2> vertices;

};