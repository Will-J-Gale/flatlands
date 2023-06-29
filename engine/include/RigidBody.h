#pragma once

#include <Vector2.h>
#include <Transform.h>
#include <collision/colliders/Collider.h>

class RigidBody
{
public:
    RigidBody();
    RigidBody(Collider* collider);
    RigidBody(Collider* collider, float mass, float friction, float elasticity, bool isStatic=false);
    void addCollider(Collider* collider);
    void addForce(Vector2 force);
    void addAngularForce(float angularForce);
    // void setFriction(float friction);
    void setStatic(bool isStatic);
    void setMass(float mass);
    void setRestitution(float elasticity);

    Vector2 force = Vector2(0,0);
    Vector2 velocity = Vector2(0,0);
    float angularAcceleration = 0;
    float angularVelocity = 0;

    Transform transform;
    Collider* collider;
    float mass = 1.0f;
    float invMass = 1.0f;
    float rotationalInertia = 1.0f;
    float invRotationalInertia = 1.0f;
    float staticFriction = 0.6f;
    float dynamicFriction = 0.4f;
    float angularDampening = 0.01f;
    bool isStatic = false;
    float restitution = 1.0f;
};