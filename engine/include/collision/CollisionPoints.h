#pragma once
#include <Vector2.h>
#include <RigidBody.h>
#include <vector>

struct CollisionPoints 
{
    std::vector<Vector2> contacts;
    Vector2 normal;
    float depth = 0.0f;
    bool hasCollisions = false;

    Vector2 impulses[2];
    Vector2 frictionImpulses[2];
};