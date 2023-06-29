#include <collision/colliders/BoxCollider.h>
#include <collision/CollisionAlgorithms.h>
#include <core/Math.h>
#include <core/Logger.h>

BoxCollider::BoxCollider()
{

}
BoxCollider::BoxCollider(float width, float height)
{
    this->width = width;
    this->height = height;

    this->halfWidth = width / 2.0f;
    this->halfHeight = height / 2.0f;
}

std::vector<Vector2> BoxCollider::transformPoints(Transform* transform)
{
    Vector2 position = transform->position;
    float radians = transform->rotation;

    //@TODO precalculate box point so you only need to apply rotation + translation
    Vector2 topLeft =   Vector2(-halfWidth, -halfHeight).rotate(radians);
    Vector2 topRight =  Vector2(halfWidth, -halfHeight).rotate(radians);
    Vector2 bottomLeft = Vector2(-halfWidth, halfHeight).rotate(radians);
    Vector2 bottomRight = Vector2(halfWidth, halfHeight).rotate(radians);

    return {
        topLeft + position, 
        topRight + position, 
        bottomRight + position, 
        bottomLeft + position
    };
}

std::vector<Vector2> BoxCollider::getAxes(float radians)
{
    Vector2 topLeft =   Vector2(-halfWidth, -halfHeight).rotate(radians);
    Vector2 topRight =  Vector2(halfWidth, -halfHeight).rotate(radians);
    Vector2 bottomRight = Vector2(halfWidth, halfHeight).rotate(radians);

    Vector2 axis1 = (topRight - topLeft).normalize();
    Vector2 axis2 = (bottomRight - topRight).normalize(); //Normally you would go clockwise for normals but because Y is flipped in this program it makes it weird

    return {axis1, axis2};
}

AABBCollider BoxCollider::GetAABB(Transform* transform)
{
    Vector2 min = Vector2(Math::FLOAT_MAX, Math::FLOAT_MAX);
    Vector2 max = Vector2(Math::FLOAT_MIN, Math::FLOAT_MIN);

    std::vector<Vector2> transformedPoints = transformPoints(transform);

    for(Vector2& point : transformedPoints)
    {
        if(point.x < min.x)
            min.x = point.x;

        if(point.x > max.x)
            max.x = point.x;

        if(point.y < min.y)
            min.y = point.y;

        if(point.y > max.y)
            max.y = point.y;
    }

    return AABBCollider(min, max);
};

std::vector<Line> BoxCollider::getEdges(Transform* transform)
{
    std::vector<Vector2> vertices = transformPoints(transform);
    std::vector<Line> edges;

    for(int i = 0; i < vertices.size(); i++)
    {
        Vector2 v1 = vertices[i];
        Vector2 v2 = vertices[(i + 1) % vertices.size()];

        edges.push_back(Line(v1, v2));
    }

    return edges;
}

float BoxCollider::GetRotationalInertia(float mass)
{
    return (1.0f/12.0f) * mass * ((height*height) + (width*width));
}