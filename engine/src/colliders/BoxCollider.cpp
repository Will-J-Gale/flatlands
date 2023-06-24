#include <colliders/BoxCollider.h>
#include <colliders/CollisionAlgorithms.h>
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

std::vector<Vector2> BoxCollider::transformPoints(Vector2 position, float radians)
{
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

    std::vector<Vector2> transformedPoints = transformPoints(transform->position, transform->rotation);

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