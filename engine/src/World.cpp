#include "World.h"
#include <colliders/Collision.h>
#include <colliders/CollisionPoints.h>
#include <core/Logger.h>
#include <algorithm>

World::World(Vector2 gravity)
{
    this->gravity = gravity;
    collisions = std::vector<Collision>();
}

void World::step(float dt)
{
    resolveCollisions();
    
    for(RigidBody* body : bodies)
    {
        if(body->isStatic)
            continue;

        body->force += this->gravity * dt;
        body->velocity += body->force * dt;
        body->velocity *= 1 - body->friction;
        body->transform.position += body->velocity * dt;

        body->angularVelocity += body->angularAcceleration * dt;
        body->angularVelocity *= 1 - body->angularDampening * dt;
        body->transform.rotation += (body->angularVelocity * dt);

        body->force.set(0.0f, 0.0f);
        body->angularAcceleration = 0.0f;
    }
}

void World::resolveCollisions()
{
    //Detect collisions
    collisions = std::vector<Collision>();

    for(int i = 0; i < bodies.size(); i++)
    {
        for(int j = 0; j < bodies.size(); j++)
        {
            if(i == j || j < i)
                continue;

            RigidBody* a = bodies[i];
            RigidBody* b = bodies[j];

            CollisionPoints collisionPoints = 
                a->collider->TestCollision(&a->transform, b->collider, &b->transform);

            if(collisionPoints.hasCollisions)
            {
                Collision collision {
                    a,
                    b,
                    collisionPoints
                };
                collisions.push_back(collision);
            }
        }
    }

    //Resolve collisions
    for(Collision& collision : collisions)
    {
        //Penetration resolution
        RigidBody* a = collision.a;
        RigidBody* b = collision.b;
        CollisionPoints collisionPoints = collision.collisionPoints;

        Vector2 penetrationResolution = collisionPoints.normal * collisionPoints.depth;

        if(b->isStatic)
            a->transform.position += -penetrationResolution;
        else if(a->isStatic)
            b->transform.position += penetrationResolution;
        else
        {
            a->transform.position += -penetrationResolution / 2;
            b->transform.position += penetrationResolution / 2;
        }
        
        //Collision resolition v1
        float totalMass = a->invMass + b->invMass;
        Vector2 relativeVelocity = a->velocity - b->velocity;
        float separatingVelocity = Vector2::dot(relativeVelocity, collisionPoints.normal);
        float newSeparatingVelocity = -separatingVelocity * std::min(a->restitution, b->restitution);
        float separatingVelocityDiff = newSeparatingVelocity - separatingVelocity;
        float impulse = separatingVelocityDiff / totalMass;
        Vector2 separatingVelocityVector = collisionPoints.normal * impulse;
        
        if(!a->isStatic)
            a->velocity += (separatingVelocityVector * a->restitution) * a->invMass;
        
        if(!b->isStatic)
            b->velocity += (-separatingVelocityVector * b->restitution) * b->invMass;
    }
}

void World::addRigidBody(RigidBody* rigidBody)
{
    bodies.push_back(rigidBody);
}

std::vector<RigidBody*> World::getBodies()
{
    return bodies;
}

std::vector<Collision>* World::getCollisions()
{
    return &collisions;
}