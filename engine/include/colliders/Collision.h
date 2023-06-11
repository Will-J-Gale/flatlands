#pragma once
#include <RigidBody.h>
#include <colliders/CollisionPoints.h>

struct Collision
{
    RigidBody* a;
    RigidBody* b;
    CollisionPoints collisionPoints;
};