#include <cmath>
#include "RigidBody.h"
#include "core/Logger.h"

RigidBody::RigidBody()
{
    
}

RigidBody::RigidBody(Collider* collider)
{
    this->collider = collider;
}

RigidBody::RigidBody(Collider* collider,  float mass, float friction, float elasticity, bool isStatic)
{
    this->collider = collider;
    this->setMass(mass);
    // this->setFriction(friction);
    this->setStatic(isStatic);
    this->setRestitution(elasticity);
}

void RigidBody::addCollider(Collider* collider)
{
    this->collider = collider;
}

void RigidBody::addForce(Vector2 force)
{
    this->force += force / mass;
}

void RigidBody::addAngularForce(float angularForce)
{
    this->angularAcceleration += angularForce / mass;
}

// void RigidBody::setFriction(float friction)
// {
//     this->friction = friction;
// }

void RigidBody::setStatic(bool isStatic)
{
    this->isStatic = isStatic;
    this->setMass(this->mass);
}

void RigidBody::setMass(float mass)
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

void RigidBody::setRestitution(float elasticity)
{
    this->restitution = elasticity;
}
