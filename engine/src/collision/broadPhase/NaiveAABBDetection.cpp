#include <collision/broadPhase/NaiveAABBDetection.h>
#include <collision/CollisionAlgorithms.h>

BroadPhaseResult NaiveAABBDetection::execute(const RigidBodies& bodies)
{
    double start = Time::time();
    BroadPhaseResult result;

    for(int i = 0; i < bodies.size(); i++)
    {
        for(int j = 0; j < bodies.size(); j++)
        {
            RigidBody* a = bodies[i];
            RigidBody* b = bodies[j];

            if(i == j || j < i)
                continue;
            
            if(a->isStatic && b->isStatic)
            continue;
        
            if(!a->isAwake && !b->isAwake)
                continue;
            
            if(!a->isAwake && b->isStatic)
                continue;
            
            if(a->isStatic && !b->isAwake)
                continue;

            result.numChecks += 1;

            AABBCollider aAABB = a->collider->GetAABB(&a->transform);
            AABBCollider bAABB = b->collider->GetAABB(&b->transform);

            if(CollisionAlgorithms::TestAABBCollision(&aAABB, &bAABB))
                result.potentialCollisions.emplace_back(a, b);
        }
    }

    result.timeTaken = Time::time() - start;
    return result;
}