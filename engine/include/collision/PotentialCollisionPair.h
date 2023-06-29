#pragma once
#include <RigidBody.h>

struct PotentialCollisionPair
{
    PotentialCollisionPair(RigidBody* a, RigidBody* b)
    {
        this->a = a;
        this->b = b;
    }

    RigidBody* a;
    RigidBody* b;
};