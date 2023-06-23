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
        body->force += this->gravity * body->mass;
        body->velocity += body->force;
        body->velocity *= 1 - body->friction;
        body->transform.position += (body->velocity * dt);

        body->angularVelocity += body->angularAcceleration;
        body->angularVelocity *= 1 - body->angularDampening;
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

        Vector2 direction = (collisionPoints.a - collisionPoints.b).normalize();
        float totalMass = a->invMass + b->invMass;
        Vector2 penetrationResolution = collisionPoints.normal * (collisionPoints.depth / 2.0f);
        a->transform.position += -penetrationResolution;
        b->transform.position += penetrationResolution;

        // if(!a->isStatic)
        //     a->transform.position += -(penetrationResolution * a->invMass);
        
        // if(!b->isStatic)
        //     b->transform.position += penetrationResolution * b->invMass;

        // //Collision resolition
        // Vector2 relativeVelocity = a->velocity - b->velocity;
        // float separatingVelocity = Vector2::dot(relativeVelocity, direction);
        // float newSeparatingVelocity = -separatingVelocity * std::min(a->elasticity, b->elasticity);
        // float separatingVelocityDiff = newSeparatingVelocity - separatingVelocity;
        // float impulse = separatingVelocityDiff / totalMass;
        // Vector2 separatingVelocityVector = direction * impulse;
        
        // if(!a->isStatic)
        //     a->velocity += (separatingVelocityVector * a->elasticity) * a->invMass;
        
        // if(!b->isStatic)
        //     b->velocity += (-separatingVelocityVector * b->elasticity) * b->invMass;
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