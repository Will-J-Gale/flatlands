#include <vector>
#include <functional>
#include <collision/PotentialCollisionPair.h>
#include <collision/CollisionAlgorithms.h>
#include <core/Time.h>
#include <RigidBody.h>

typedef std::vector<PotentialCollisionPair> PotentialCollisions;
struct BroadPhaseResult
{
    PotentialCollisions potentialCollisions;
    unsigned int numChecks = 0;
    double timeTaken;
};

typedef std::vector<RigidBody*> RigidBodies;
typedef std::function<BroadPhaseResult(const RigidBodies& bodies)> BroadPhaseDetection;

namespace BroadPhase
{
    inline BroadPhaseResult NaiveAABBDetection(const RigidBodies& bodies)
    {
        double start = Time::time();
        BroadPhaseResult result;

        for(int i = 0; i < bodies.size(); i++)
        {
            for(int j = 0; j < bodies.size(); j++)
            {
                RigidBody* a = bodies[i];
                RigidBody* b = bodies[j];

                if(i == j || j < i || (a->isStatic && b->isStatic))
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
}