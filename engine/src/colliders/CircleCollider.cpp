#include <colliders/CircleCollider.h>
#include <colliders/CollisionAlgorithms.h>

CircleCollider::CircleCollider() : Collider()
{
    
}

CircleCollider::CircleCollider(float radius) : Collider()
{
    this->radius = radius;
}

AABBCollider CircleCollider::GetAABB(Transform* transform)
{
    Vector2 min = transform->position - Vector2(radius, radius);
    Vector2 max = transform->position + Vector2(radius, radius);

    return AABBCollider(min, max);
}