#pragma once

#include <Vector2.h>
#include <Transform.h>
#include <colliders/Collider.h>

class RigidBody
{
public:
    RigidBody();
    RigidBody(Collider* collider);
    RigidBody(Collider* collider, float mass, float friction, float elasticity, bool isStatic=false);
    void addCollider(Collider* collider);
    void addForce(Vector2 force);
    void addAngularForce(float angularForce);
    void setFriction(float friction);
    void setStatic(bool isStatic);
    void setMass(float mass);
    void setElasticity(float elasticity);

    Vector2 force = Vector2(0,0);
    Vector2 velocity = Vector2(0,0);
    float angularAcceleration = 0;
    float angularVelocity = 0;

    Transform transform;
    Collider* collider;
    float mass = 1;
    float friction = 0.05;
    float angularDampening = 0.05;
    bool isStatic = false;
    float elasticity = 1;
    float invMass = 1;
};