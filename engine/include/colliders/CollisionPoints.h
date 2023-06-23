#pragma once
#include <Vector2.h>
#include <RigidBody.h>
#include <vector>

struct CollisionPoints 
{
    Vector2 a; //Furthest point of A into B
    Vector2 b; //Furthest point of B into A
    Vector2 normal;
    float depth;
    bool hasCollisions = false;
};