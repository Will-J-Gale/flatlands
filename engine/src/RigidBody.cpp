#include "RigidBody.h"

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
    this->setFriction(friction);
    this->setStatic(isStatic);
    this->setElasticity(elasticity);
}


void RigidBody::addCollider(Collider* collider)
{
    this->collider = collider;
}

void RigidBody::addForce(Vector2 force)
{
    this->force += force;
}

void RigidBody::addAngularForce(float angularForce)
{
    this->angularAcceleration += angularForce;
}

void RigidBody::setFriction(float friction)
{
    this->friction = friction;
}

void RigidBody::setStatic(bool isStatic)
{
    this->isStatic = isStatic;
}

void RigidBody::setMass(float mass)
{
    this->mass = mass;
    this->invMass = mass > 0 ? 1 / mass : 0;
}

void RigidBody::setElasticity(float elasticity)
{
    this->elasticity = elasticity;
}
