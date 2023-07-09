#pragma once

#include <vector>
#include <RigidBody.h>
#include <Vector2.h>
#include <collision/Collision.h>
#include <collision/PotentialCollisionPair.h>
#include <core/Metrics.h>

class World
{
public:
    World(Vector2 gravity, int num_iterations);
    void step(float dt);
    void addRigidBody(RigidBody* rigidBody);
    std::vector<RigidBody*> getBodies();
    std::vector<Collision>* getCollisions();
    Metrics getMetrics() { return metrics; }
    
private:
    void subStep(float dt);
    void moveRigidBodies(float dt);
    std::vector<PotentialCollisionPair> broadPhaseDetection();
    void narrowPhaseDetection(std::vector<PotentialCollisionPair> potentialCollisions);
    void resolveCollisions();

    std::vector<RigidBody*> bodies;
    std::vector<Collision> collisions;
    Vector2 gravity;
    int numIterations = 1;

    Metrics metrics;
};