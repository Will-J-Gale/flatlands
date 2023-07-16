#pragma once

#include <string>
#include <math/Vector2.h>
#include <Transform.h>
#include <collision/colliders/Collider.h>
#include <core/EngineConstants.h>

class RigidBody
{
public:
    RigidBody();
    RigidBody(Collider* collider);
    RigidBody(Collider* collider, float mass, float friction, float elasticity, bool isStatic=false);
    void AddCollider(Collider* collider);
    void ApplyGravity(Vector2 gravity);
    void AddForce(Vector2 force);
    void AddAngularForce(float angularForce);
    // void setFriction(float friction);
    void SetStatic(bool isStatic);
    void SetMass(float mass);
    void SetRestitution(float elasticity);
    void Step(float dt);
    void CheckAwake();
    void WakeUp();

public:
    Transform transform;
    Collider* collider;
    Vector2 force = Vector2(0,0);
    Vector2 velocity = Vector2(0,0);
    float angularAcceleration = 0;
    float angularVelocity = 0;
    float mass = DEFAULT_MASS;
    float invMass = 1.0f;
    float rotationalInertia = DEFAULT_ROTATIONAL_INERTIA;
    float invRotationalInertia = 1.0f;
    float staticFriction = DEFAULT_STATIC_FRICTION;
    float dynamicFriction = DEFAULT_DYNAMIC_FRICTION;
    float angularDampening = DEFAULT_ANGULAR_DAMPENING;
    bool isStatic = false;
    float restitution = DEFAULT_RESTITUTION;
    bool isAwake = true;
    std::string hash;

private:
    void GenerateHash();

private:
    Vector2 previousPosition;
    int notMovingCounter = 0;
};