#pragma once

#include <memory>
#include <vector>
#include <RigidBody.h>
#include <math/Vector2.h>
#include <collision/Collision.h>
#include <collision/PotentialCollisionPair.h>
#include <collision/broadPhase/BroadPhaseDetection.h>
#include <core/Metrics.h>

class World
{
public:
    World(Vector2 gravity, int num_iterations);
    void Step(float dt);
    void AddRigidBody(RigidBody* rigidBody);
    std::vector<RigidBody*> GetBodies();
    std::vector<Collision>* GetCollisions();
    Metrics GetMetrics() { return metrics; }
    void SetBroadPhase(std::unique_ptr<BroadPhaseDetection> broadPhase);
    
private:
    void SubStep(float dt);
    void MoveRigidBodies(float dt);
    void NarrowPhaseDetection(std::vector<PotentialCollisionPair> potentialCollisions);
    void ResolveCollisions();

private:
    int numIterations = 1;
    std::vector<RigidBody*> bodies;
    std::vector<Collision> collisions;
    Vector2 gravity;
    Metrics metrics;
    std::unique_ptr<BroadPhaseDetection> broadPhase = nullptr;
};