#include <cmath>
#include <sstream>
#include "RigidBody.h"
#include "core/Logger.h"

RigidBody::RigidBody()
{
    this->GenerateHash();
}

RigidBody::RigidBody(Collider* collider)
{
    this->collider = collider;
    this->GenerateHash();
}

RigidBody::RigidBody(Collider* collider,  float mass, float friction, float elasticity, bool isStatic)
{
    this->collider = collider;
    this->SetMass(mass);
    // this->setFriction(friction);
    this->SetStatic(isStatic);
    this->SetRestitution(elasticity);
    this->GenerateHash();
}

void RigidBody::GenerateHash()
{
    std::ostringstream addressStream;
    addressStream << this;
    this->hash = addressStream.str();
}

void RigidBody::AddCollider(Collider* collider)
{
    this->collider = collider;
}

void RigidBody::ApplyGravity(Vector2 gravity)
{
    if(isAwake)
        this->velocity += gravity;
}

void RigidBody::AddForce(Vector2 force)
{
    WakeUp();
    this->force += force * invMass;
}

void RigidBody::AddAngularForce(float angularForce)
{
    isAwake = true;
    notMovingCounter = 0;
    this->angularAcceleration += angularForce / mass;
}

// void RigidBody::setFriction(float friction)
// {
//     this->friction = friction;
// }

void RigidBody::SetStatic(bool isStatic)
{
    this->isStatic = isStatic;
    this->SetMass(this->mass);
}

void RigidBody::SetMass(float mass)
{
    this->mass = std::max(mass, 0.0f);
    this->invMass = mass > 0 ? 1 / mass : 0.0f;
    
    this->rotationalInertia = collider->GetRotationalInertia(this->mass);
    this->invRotationalInertia = this->rotationalInertia > 0 ? 1.0f / rotationalInertia : 0.0f;

    if(this->mass == 0.0f)
    {
        this->isStatic = true;
        this->rotationalInertia = 0.0f;
    }

    if(this->isStatic)
    {
        this->invMass = 0.0f;
        this->invRotationalInertia = 0.0f;
    }
}

void RigidBody::SetRestitution(float elasticity)
{
    this->restitution = elasticity;
}

void RigidBody::Step(float dt)
{
    if(!isAwake || isStatic)
        return;
    
    previousPosition = transform.position;

    velocity += dt * (force * invMass);
    transform.position += velocity * dt;

    angularVelocity += angularAcceleration * invRotationalInertia * dt;
    transform.rotation += angularVelocity * dt;

    force.set(0.0f, 0.0f);
    angularAcceleration = 0.0f;
}

void RigidBody::CheckAwake()
{
    if(isStatic || !isAwake)
        return;

    float movement = (transform.position - previousPosition).magnitude();

    if(movement < NOT_MOVING_THRESHOLD)
        notMovingCounter++;

    if(notMovingCounter >= SLEEP_COUNT)
        isAwake = false;

    previousPosition = transform.position;
}

void RigidBody::WakeUp()
{
    isAwake = true;
    notMovingCounter = 0;
}