#pragma once
#include <Vector2.h>
#include <RigidBody.h>
#include <vector>

struct CollisionPoints 
{
    int numContacts = 0;
    Vector2 contactA; //Furthest point of A into B
    Vector2 contactB; //Furthest point of B into A
    Vector2 normal;
    float depth = 0.0f;
    bool hasCollisions = false;
};