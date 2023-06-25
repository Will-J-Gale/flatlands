#include "World.h"
#include <algorithm>
#include <colliders/Collision.h>
#include <colliders/CollisionPoints.h>
#include <core/Logger.h>
#include <colliders/CollisionAlgorithms.h>
#include <core/Timer.h>
#include <cmath>
#include <core/Math.h>

World::World(Vector2 gravity, int numIterations)
{
    this->gravity = gravity;
    this->numIterations = numIterations;
    collisions = std::vector<Collision>();
}

void World::step(float dt)
{
    Timer::start("World Step");

    float dtFraction = dt / numIterations;
    for(int i = 0; i < numIterations; i++)
    {
        subStep(dtFraction);
    }

    Timer::stop("World Step");
}

void World::subStep(float dt)
{
    for(RigidBody* body : bodies)
    {
        if(body->isStatic)
            continue;

        body->velocity += dt * this->gravity + (body->force * body->invMass);
        body->transform.position += body->velocity * dt;

        body->angularVelocity += body->angularAcceleration * body->invRotationalInertia * dt;
        body->transform.rotation += body->angularVelocity * dt;

        body->force.set(0.0f, 0.0f);
        body->angularAcceleration = 0.0f;
    }

    collisions.clear();
    std::vector<PotentialCollisionPair> potentialCollisions = broadPhaseDetection();
    narrowPhaseDetection(potentialCollisions);
    resolveCollisions();
}

std::vector<PotentialCollisionPair> World::broadPhaseDetection()
{
    std::vector<PotentialCollisionPair> potentialCollisions;

    for(int i = 0; i < bodies.size(); i++)
    {
        for(int j = 0; j < bodies.size(); j++)
        {
            if(i == j || j < i)
                continue;

            RigidBody* a = bodies[i];
            RigidBody* b = bodies[j];
            AABBCollider aAABB = a->collider->GetAABB(&a->transform);
            AABBCollider bAABB = b->collider->GetAABB(&b->transform);

            if(CollisionAlgorithms::TestAABBCollision(&aAABB, &bAABB))
                potentialCollisions.emplace_back(a, b);
        }
    }

    return potentialCollisions;
}
void World::narrowPhaseDetection(std::vector<PotentialCollisionPair> potentialCollisions)
{
    for(PotentialCollisionPair& potentialCollision : potentialCollisions)
    {
        RigidBody* a = potentialCollision.a;
        RigidBody* b = potentialCollision.b;

        if(a->isStatic && b->isStatic)
            continue;

        CollisionPoints collisionPoints = CollisionAlgorithms::TestCollision(
            a->collider, &a->transform,
            b->collider, &b->transform
        );

        if(collisionPoints.hasCollisions)
        {
            //Resolve penetration
            Vector2 penetrationResolution = collisionPoints.normal * collisionPoints.depth;

            if(b->isStatic)
                a->transform.position += -penetrationResolution;
            else if(a->isStatic)
                b->transform.position += penetrationResolution;
            else
            {
                a->transform.position += (-penetrationResolution / 2.0f);
                b->transform.position += (penetrationResolution / 2.0f);
            }

            Collision collision = Collision(a, b, collisionPoints);
            CollisionAlgorithms::GenerateContactPoints(collision);
            collisions.emplace_back(std::move(collision));
        }
    }
}

void World::resolveCollisions()
{
    //Resolve collisions
    for(Collision& collision : collisions)
    {
        RigidBody* a = collision.a;
        RigidBody* b = collision.b;
        CollisionPoints* collisionPoints = &collision.collisionPoints;
        float e = std::min(a->restitution, b->restitution);
        float staticFriction = (a->staticFriction + b->staticFriction) * 0.5f;
        float dynamicFriction = (a->dynamicFriction + b->dynamicFriction) * 0.5f;
        
        Vector2 impulses[2];
        Vector2 frictionImpulses[2];
        float impulseMagnitudes[2] = {};

        //Resolve impulses
        for(int i = 0; i < collisionPoints->contacts.size(); i++)
        {
            Vector2 contact = collisionPoints->contacts[i];
            Vector2 ra = contact - a->transform.position;
            Vector2 rb = contact - b->transform.position;
            Vector2 raPerp = ra.perpendicular();
            Vector2 rbPerp = rb.perpendicular();

            Vector2 aAngularVelocityVector = a->angularVelocity * raPerp;
            Vector2 bAngularVelocityVector = b->angularVelocity * rbPerp;

            Vector2 relativeVelocity = 
                (b->velocity + bAngularVelocityVector) - 
                (a->velocity + aAngularVelocityVector);
            
            float contactVelocityMagnitude = Vector2::dot(relativeVelocity, collisionPoints->normal);

            if (contactVelocityMagnitude > 0.0f)
                continue;

            float raPerpDotNormal = Vector2::dot(raPerp, collisionPoints->normal);
            float rbPerpDotNormal = Vector2::dot(rbPerp, collisionPoints->normal);
            
            float denominator = a->invMass + b->invMass +
                ((raPerpDotNormal * raPerpDotNormal) * a->invRotationalInertia) +
                ((rbPerpDotNormal * rbPerpDotNormal) * b->invRotationalInertia);

            float j = -(1.0f + e) * contactVelocityMagnitude;
            j /= denominator;
            j /= (float)collisionPoints->contacts.size();

            impulseMagnitudes[i] = j;
            impulses[i] = j * collisionPoints->normal;
        }

        for(int i = 0; i < collisionPoints->contacts.size(); i++)
        {
            Vector2 contact = collisionPoints->contacts[i];
            Vector2 impulse = impulses[i];
            Vector2 ra = contact - a->transform.position;
            Vector2 rb = contact - b->transform.position;

            a->velocity += -impulse * a->invMass;
            a->angularVelocity += -Vector2::cross(ra, impulse) * a->invRotationalInertia;
            b->velocity += impulse * b->invMass;
            b->angularVelocity += Vector2::cross(rb, impulse) * b->invRotationalInertia;
        }

        //Apply Friction
        for(int i = 0; i < collisionPoints->contacts.size(); i++)
        {
            //Get vector points from center of object to contact
            Vector2 contact = collisionPoints->contacts[i];
            Vector2 ra = contact - a->transform.position;
            Vector2 rb = contact - b->transform.position;
            Vector2 raPerp = ra.perpendicular();
            Vector2 rbPerp = rb.perpendicular();
            Vector2 aAngularVelocityVector = a->angularVelocity * raPerp;
            Vector2 bAngularVelocityVector = b->angularVelocity * rbPerp;

            Vector2 relativeVelocity = 
                (b->velocity + bAngularVelocityVector) - 
                (a->velocity + aAngularVelocityVector);
            
            Vector2 tangent = relativeVelocity - (Vector2::dot(relativeVelocity, collisionPoints->normal) * collisionPoints->normal);
            
            if(Vector2::nearlyEqual(tangent, Vector2()))
                continue;
            else
                tangent = tangent.normalize();

            float raPerpDotTangent = Vector2::dot(raPerp, tangent);
            float rbPerpDotTangent = Vector2::dot(rbPerp, tangent);
            
            float denominator = a->invMass + b->invMass +
                ((raPerpDotTangent * raPerpDotTangent) * a->invRotationalInertia) +
                ((rbPerpDotTangent * rbPerpDotTangent) * b->invRotationalInertia);

            float jTangent = -Vector2::dot(relativeVelocity, tangent);
            jTangent /= denominator;
            jTangent /= (float)collisionPoints->contacts.size();

            float impulseMagnitude = impulseMagnitudes[i];
            Vector2 frictionImpulse = Vector2();

            if(std::abs(jTangent) <= impulseMagnitude * staticFriction)
                frictionImpulse = jTangent * tangent;
            else
                frictionImpulse = -impulseMagnitude * tangent * dynamicFriction;

            
            frictionImpulses[i] = frictionImpulse;
        }

        for(int i = 0; i < collisionPoints->contacts.size(); i++)
        {
            Vector2 contact = collisionPoints->contacts[i];
            Vector2 frictionImpulse = frictionImpulses[i];
            Vector2 ra = contact - a->transform.position;
            Vector2 rb = contact - b->transform.position;

            a->velocity += -frictionImpulse * a->invMass;
            a->angularVelocity += -Vector2::cross(ra, frictionImpulse) * a->invRotationalInertia;
            b->velocity += frictionImpulse * b->invMass;
            b->angularVelocity += Vector2::cross(rb, frictionImpulse) * b->invRotationalInertia;
        }

        //DEBUG
        for(int i = 0; i < collisionPoints->contacts.size(); i++)
        {
            collisionPoints->impulses[i] = impulses[i];
            collisionPoints->frictionImpulses[i] = frictionImpulses[i];
        }
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