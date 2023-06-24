#pragma once
#include <RigidBody.h>
#include <colliders/CollisionPoints.h>

struct Collision
{
    Collision(RigidBody* a, RigidBody* b, CollisionPoints collisionPoints)
    {
        this->a = a;
        this->b = b;
        this->collisionPoints = collisionPoints;
    }

    RigidBody* a;
    RigidBody* b;
    CollisionPoints collisionPoints;
};