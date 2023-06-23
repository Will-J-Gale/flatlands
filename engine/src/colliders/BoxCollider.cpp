#include <colliders/BoxCollider.h>
#include <colliders/CollisionAlgorithms.h>
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

CollisionPoints BoxCollider::TestCollision(
		Transform* transform,
		Collider* collider,
		Transform* colliderTransform)
{
    return collider->TestCollision(colliderTransform, this, transform);
}

CollisionPoints BoxCollider::TestCollision(
		Transform* transform,
		CircleCollider* circleCollider,
		Transform* circleTransform)
{
    return CollisionAlgorithms::FindBoxCircleCollision(
        this, transform,
        circleCollider, circleTransform
    );
}

CollisionPoints BoxCollider::TestCollision(
		Transform* transform,
		LineCollider* lineCollider,
		Transform* lineTransform)
{
    return CollisionAlgorithms::FindLineBoxCollisionPoints(
        lineCollider, lineTransform,
        this, transform
    );
}

CollisionPoints BoxCollider::TestCollision(
		Transform* transform,
		BoxCollider* boxCollider,
		Transform* boxTransform)
{
    return CollisionAlgorithms::FindBoxBoxCollision(
        boxCollider, boxTransform,
        this, transform
    );
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